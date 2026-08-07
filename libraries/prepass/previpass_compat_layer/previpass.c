/**
 * @file  previpass.c
 * @brief PREPAS v3.4 API on top of the Kineis PREPAS v4.0 engine.
 *
 * This layer does three things the v4.0 engine no longer does:
 *  1. filters satellites on their per-direction payload status BEFORE the
 *     engine runs (the "_with_status" functions);
 *  2. converts the engine result array into the v3.4 sorted linked list,
 *     with epochs in UNIX time;
 *  3. tracks the transceiver transitions across covisible satellites.
 *
 * The engine files (prepas.c, utils.c) are compiled unmodified.
 * All storage is static: no heap. The engine itself uses global variables,
 * so only one computation may run at a time (no reentrancy).
 */

#include <string.h>
#include <math.h>
#include <stddef.h>

#include "previpass.h"

/* The engine date routine is reused instead of shipping a second calendar
 * implementation: su_date_jmahms_stu20() is already linked in (prepas.c
 * calls it) and both implementations were verified to agree exactly. */

/* The engine header redefines MIN/MAX with a different spelling. */
#undef MIN
#undef MAX
#include "prepas.h"
#include "utils.h"


/* ------------------------------------------------------------------------ */
/* Build-time configuration (override with -D)                              */
/* ------------------------------------------------------------------------ */

/** Capacity of the pass list. RAM cost: sizeof(element) per pass. */
#ifndef PREVIPASS_MAX_PASSES
#define PREVIPASS_MAX_PASSES 64
#endif

/** Max number of satellites handed to the engine (engine limit: 30). */
#ifndef PREVIPASS_MAX_SATS
#define PREVIPASS_MAX_SATS NB_MAX_SAT
#endif

#if PREVIPASS_MAX_SATS > NB_MAX_SAT
#error "PREVIPASS_MAX_SATS exceeds the engine limit NB_MAX_SAT"
#endif
#if PREVIPASS_MAX_PASSES > NB_MAX_VISI
#error "PREVIPASS_MAX_PASSES exceeds the engine limit NB_MAX_VISI"
#endif


/* ------------------------------------------------------------------------ */
/* Static data                                                              */
/* ------------------------------------------------------------------------ */

/** AOP table handed to the engine (satellites kept after filtering). */
static st_aop aop_scratch[PREVIPASS_MAX_SATS];

/** For each engine satellite, its index in the caller AOP table. */
static uint8_t aop_index_map[PREVIPASS_MAX_SATS];

/** Raw engine results. */
static st_res engine_results[PREVIPASS_MAX_PASSES];

/** Pass list storage; replaces the v3.4 byte pool. */
static struct SatPassLinkedListElement_t pass_pool[PREVIPASS_MAX_PASSES];

/** Number of valid entries in pass_pool. */
static uint16_t pass_count;

/* State of PREVIPASS_process_existing_sorted_passes: what was in sight at
 * the previous call. */
static uint8_t prev_uplink_status;
static uint8_t prev_downlink_status;


/* ------------------------------------------------------------------------ */
/* Private prototypes                                                       */
/* ------------------------------------------------------------------------ */

static uint8_t build_engine_aop_table(const struct AopSatelliteEntry_t *aopTable,
		uint8_t nbSats, uint8_t downlinkMin, uint8_t uplinkMin);

static struct SatPassLinkedListElement_t *run_engine(
		const struct PredictionPassConfiguration_t *config,
		const struct AopSatelliteEntry_t *aopTable, uint8_t nbSats,
		uint8_t downlinkMin, uint8_t uplinkMin, bool *overflow);

static enum TransceiverAction_t transceiver_action(uint8_t prevUplink,
		uint8_t prevDownlink, uint8_t uplink, uint8_t downlink);


/* ------------------------------------------------------------------------ */
/* Public function: default AOP entry                                       */
/* ------------------------------------------------------------------------ */

struct AopSatelliteEntry_t PREVIPASS_default_aop_satellite_entry(void)
{
	struct AopSatelliteEntry_t entry;

	(void)memset(&entry, 0, sizeof(entry));
	entry.downlinkStatus = (uint8_t)SAT_DNLK_OFF;
	entry.uplinkStatus   = (uint8_t)SAT_UPLK_OFF;

	return entry;
}


/* ------------------------------------------------------------------------ */
/* Public functions: pass computation                                       */
/* ------------------------------------------------------------------------ */

struct SatPassLinkedListElement_t *PREVIPASS_compute_new_prediction_pass_times(
	struct PredictionPassConfiguration_t *config,
	struct AopSatelliteEntry_t           *aopTable,
	uint8_t                               nbSatsInAopTable,
	bool                                 *memoryPoolOverflow
)
{
	return PREVIPASS_compute_new_prediction_pass_times_with_status(config,
			aopTable, nbSatsInAopTable,
			SAT_DNLK_OFF, SAT_UPLK_OFF, memoryPoolOverflow);
}


struct SatPassLinkedListElement_t *
PREVIPASS_compute_new_prediction_pass_times_with_status(
	struct PredictionPassConfiguration_t *config,
	struct AopSatelliteEntry_t           *aopTable,
	uint8_t                               nbSatsInAopTable,
	uint8_t                               downlinkStatus,
	uint8_t                               uplinkStatus,
	bool                                 *memoryPoolOverflow
)
{
	return run_engine(config, aopTable, nbSatsInAopTable,
			downlinkStatus, uplinkStatus, memoryPoolOverflow);
}


bool PREVIPASS_compute_next_pass(
	struct PredictionPassConfiguration_t *config,
	struct AopSatelliteEntry_t           *aopTable,
	uint8_t                               nbSatsInAopTable,
	struct SatelliteNextPassPrediction_t *nextPass
)
{
	return PREVIPASS_compute_next_pass_with_status(config, aopTable,
			nbSatsInAopTable, SAT_DNLK_OFF, SAT_UPLK_OFF, nextPass);
}


bool PREVIPASS_compute_next_pass_with_status(
	struct PredictionPassConfiguration_t *config,
	struct AopSatelliteEntry_t           *aopTable,
	uint8_t                               nbSatsInAopTable,
	uint8_t                               downlinkStatus,
	uint8_t                               uplinkStatus,
	struct SatelliteNextPassPrediction_t *nextPass
)
{
	struct PredictionPassConfiguration_t localConfig;
	struct SatPassLinkedListElement_t *list;
	bool overflow;

	if ((config == NULL) || (nextPass == NULL))
		return false;

	/* Search window: 24 hours from the start date (v3.4 behaviour). */
	localConfig = *config;
	localConfig.end = localConfig.start;
	localConfig.end.day += 1u;
	localConfig.maxPasses = 1u;

	list = run_engine(&localConfig, aopTable, nbSatsInAopTable,
			downlinkStatus, uplinkStatus, &overflow);
	if (list == NULL)
		return false;

	*nextPass = list->element;

	return true;
}


/* ------------------------------------------------------------------------ */
/* Public function: transceiver transitions                                 */
/* ------------------------------------------------------------------------ */

struct NextPassTransceiverCapacity_t PREVIPASS_process_existing_sorted_passes(
	uint32_t                           currentTime,
	struct SatPassLinkedListElement_t *previsionPassesList
)
{
	const struct SatPassLinkedListElement_t *elt = previsionPassesList;
	struct NextPassTransceiverCapacity_t retAction = {
		UNKNOWN_TRANSCEIVER_ACTION, 0u, 0u
	};

	if (previsionPassesList == NULL)
		return retAction;

	/* Collect what the satellites in sight offer, per direction. */
	while (elt != NULL) {
		uint32_t passStart = elt->element.epoch;
		uint32_t passEnd   = elt->element.epoch + elt->element.duration;

		if ((currentTime >= passStart) && (currentTime < passEnd)) {
			/* Lowest generation everybody in sight can decode. */
			if ((retAction.minUplinkStatus == (uint8_t)SAT_UPLK_OFF)
					|| (elt->element.uplinkStatus
						< retAction.minUplinkStatus))
				retAction.minUplinkStatus = elt->element.uplinkStatus;

			if (elt->element.downlinkStatus != (uint8_t)SAT_DNLK_OFF)
				retAction.downlinkStatus = (uint8_t)SAT_DNLK_ON;
		}

		elt = elt->next;
	}

	retAction.trcvrActionForNextPass = transceiver_action(
			prev_uplink_status, prev_downlink_status,
			retAction.minUplinkStatus, retAction.downlinkStatus);

	prev_uplink_status   = retAction.minUplinkStatus;
	prev_downlink_status = retAction.downlinkStatus;

	return retAction;
}


/* ------------------------------------------------------------------------ */
/* Private functions: engine invocation                                     */
/* ------------------------------------------------------------------------ */

/**
 * Fill aop_scratch with the satellites this computation may use: stop at
 * the first null address, skip entries with no bulletin, skip satellites
 * whose two missions are OFF, and skip those that do not reach the
 * requested status in either direction.
 *
 * OFF (zero) leaves its direction unconstrained.
 *
 * @return the number of satellites kept.
 */
static uint8_t build_engine_aop_table(
	const struct AopSatelliteEntry_t *aopTable,
	uint8_t                           nbSats,
	uint8_t                           downlinkMin,
	uint8_t                           uplinkMin
)
{
	uint8_t iSat;
	uint8_t kept = 0u;

	for (iSat = 0u; (iSat < nbSats) && (aopTable[iSat].satHexId != 0u); ++iSat) {
		const struct AopSatelliteEntry_t *src = &aopTable[iSat];
		st_aop *dst;

		if (src->bulletin.year == 0u)
			continue;
		if ((src->downlinkStatus == (uint8_t)SAT_DNLK_OFF)
				&& (src->uplinkStatus == (uint8_t)SAT_UPLK_OFF))
			continue;
		if ((src->downlinkStatus < downlinkMin)
				|| (src->uplinkStatus < uplinkMin))
			continue;
		if (kept >= (uint8_t)PREVIPASS_MAX_SATS)
			break;

		dst = &aop_scratch[kept];

		/* The engine ignores the satellite name. */
		dst->sat[0] = '?';
		dst->sat[1] = '?';
		dst->sat[2] = '\0';

		dst->an_bul    = (int)src->bulletin.year;
		dst->mois_bul  = (int)src->bulletin.month;
		dst->jour_bul  = (int)src->bulletin.day;
		dst->heure_bul = (int)src->bulletin.hour;
		dst->min_bul   = (int)src->bulletin.minute;
		dst->sec_bul   = (int)src->bulletin.second;

		dst->dga     = src->semiMajorAxisKm;
		dst->inc     = src->inclinationDeg;
		dst->lon_asc = src->ascNodeLongitudeDeg;
		dst->d_noeud = src->ascNodeDriftDeg;
		dst->ts      = src->orbitPeriodMin;
		dst->dgap    = src->semiMajorAxisDriftMeterPerDay;

		aop_index_map[kept] = iSat;
		++kept;
	}

	return kept;
}


/**
 * Run the v4.0 engine and rebuild the v3.4 sorted linked list.
 * See the header for the public contract.
 */
static struct SatPassLinkedListElement_t *run_engine(
	const struct PredictionPassConfiguration_t *config,
	const struct AopSatelliteEntry_t           *aopTable,
	uint8_t                                     nbSats,
	uint8_t                                     downlinkMin,
	uint8_t                                     uplinkMin,
	bool                                       *overflow
)
{
	st_config cfg;
	uint8_t engine_sats;
	uint32_t passes_per_sat[PREVIPASS_MAX_SATS];
	uint32_t global_limit;
	uint32_t start_sec70;
	long start_sec20 = 0;
	int nb_visi = 0;
	int i;

	pass_count = 0u;
	if (overflow != NULL)
		*overflow = false;

	if ((config == NULL) || (aopTable == NULL) || (nbSats == 0u)
			|| (config->maxPasses == 0u))
		return NULL;

	/* 1. Satellite filtering (feature removed from the v4.0 engine). */
	engine_sats = build_engine_aop_table(aopTable, nbSats, downlinkMin,
			uplinkMin);
	if (engine_sats == 0u)
		return NULL;

	for (i = 0; i < (int)engine_sats; ++i)
		passes_per_sat[i] = 0u;

	/* 2. Engine configuration. */
	(void)memset(&cfg, 0, sizeof(cfg));
	cfg.pf_lat = config->beaconLatitude;
	cfg.pf_lon = config->beaconLongitude;

	cfg.an_deb    = (int)config->start.year;
	cfg.mois_deb  = (int)config->start.month;
	cfg.jour_deb  = (int)config->start.day;
	cfg.heure_deb = (int)config->start.hour;
	cfg.min_deb   = (int)config->start.minute;
	cfg.sec_deb   = (int)config->start.second;

	cfg.an_fin    = (int)config->end.year;
	cfg.mois_fin  = (int)config->end.month;
	cfg.jour_fin  = (int)config->end.day;
	cfg.heure_fin = (int)config->end.hour;
	cfg.min_fin   = (int)config->end.minute;
	cfg.sec_fin   = (int)config->end.second;

	cfg.elevation_min     = config->minElevation;
	cfg.max_elevation_max = config->maxElevation;
	cfg.duree_min         = config->minPassDurationMinute;
	cfg.marge_temporelle  = config->timeMarginMinPer6months;

	/* v4.0-only parameters, per computation. */
	cfg.min_elevation_max    = config->minCulmination;
	cfg.marge_position       = config->beaconPositionMarginKm;
	cfg.include_current_visi = config->includeCurrentPass;

	/*
	 * config->maxPasses is PER SATELLITE (v3.4); the engine limit Npass
	 * is GLOBAL. Request the smallest safe global count, then apply the
	 * per-satellite limit when converting the results.
	 */
	if (config->maxPasses > ((uint32_t)PREVIPASS_MAX_PASSES / engine_sats))
		global_limit = (uint32_t)PREVIPASS_MAX_PASSES;
	else
		global_limit = config->maxPasses * engine_sats;
	cfg.Npass = (int)global_limit;

	/* 3. Engine run. */
	if (prepas(&cfg, aop_scratch, (int)engine_sats, engine_results,
			&nb_visi) != 0) {
		if (overflow != NULL)
			*overflow = true;
		return NULL;
	}

	if (nb_visi > (int)PREVIPASS_MAX_PASSES)
		nb_visi = (int)PREVIPASS_MAX_PASSES;
	if ((uint32_t)nb_visi >= global_limit
			&& (global_limit == (uint32_t)PREVIPASS_MAX_PASSES)) {
		/* The pool bound was reached: results may be truncated. */
		if (overflow != NULL)
			*overflow = true;
	}

	/* 4. Conversion to the v3.4 list. The engine returns pass times as
	 * an offset from the configured start date, so only that start date
	 * has to be turned into an absolute UNIX timestamp. */
	(void)su_date_jmahms_stu20((long)config->start.day,
			(long)config->start.month, (long)config->start.year,
			(long)config->start.hour, (long)config->start.minute,
			(long)config->start.second, &start_sec20);
	start_sec70 = (uint32_t)(start_sec20 + PREVIPASS_EPOCH_2020_TO_1970);

	for (i = 0; i < nb_visi; ++i) {
		const st_res *res = &engine_results[i];
		const struct AopSatelliteEntry_t *sat;
		struct SatelliteNextPassPrediction_t pass;
		uint8_t engine_idx = (uint8_t)res->num_sat;
		int32_t duration_s;
		int32_t elevation;
		uint16_t pos;

		if (engine_idx >= engine_sats)
			continue;
		if (passes_per_sat[engine_idx] >= config->maxPasses)
			continue;
		++passes_per_sat[engine_idx];

		sat = &aopTable[aop_index_map[engine_idx]];

		pass.epoch = start_sec70 + (uint32_t)(int32_t)lrintf(res->delta_start);

		duration_s = (int32_t)lrintf(res->pass_duration * 60.0f);
		if (duration_s < 0)
			duration_s = 0;
		if (duration_s > (int32_t)UINT16_MAX)
			duration_s = (int32_t)UINT16_MAX;
		pass.duration = (uint16_t)duration_s;

		elevation = (int32_t)res->pass_elev_max; /* truncation, as v3.4 */
		if (elevation < 0)
			elevation = 0;
		if (elevation > 255)
			elevation = 255;
		pass.elevationMax = (uint8_t)elevation;

		pass.satHexId       = sat->satHexId;
		pass.downlinkStatus = sat->downlinkStatus;
		pass.uplinkStatus   = sat->uplinkStatus;

		/* Sorted insertion (by epoch) into the pool. */
		pos = pass_count;
		while ((pos > 0u) && (pass_pool[pos - 1u].element.epoch > pass.epoch)) {
			pass_pool[pos].element = pass_pool[pos - 1u].element;
			--pos;
		}
		pass_pool[pos].element = pass;
		++pass_count;
	}

	if (pass_count == 0u)
		return NULL;

	/* 5. Chain the elements. */
	for (i = 0; i < (int)pass_count - 1; ++i)
		pass_pool[i].next = &pass_pool[i + 1];
	pass_pool[pass_count - 1u].next = NULL;

	return &pass_pool[0];
}


/* ------------------------------------------------------------------------ */
/* Private function: transceiver decision                                   */
/* ------------------------------------------------------------------------ */

/**
 * Decide what the transceiver must do, from what was in sight before and
 * after this call.
 *
 * A direction is available when its status is not OFF. A direction needs
 * a radio reconfiguration when its status changed while staying active.
 */
static enum TransceiverAction_t transceiver_action(
	uint8_t prevUplink,
	uint8_t prevDownlink,
	uint8_t uplink,
	uint8_t downlink
)
{
	bool hadTx = (prevUplink != 0u);
	bool hadRx = (prevDownlink != 0u);
	bool hasTx = (uplink != 0u);
	bool hasRx = (downlink != 0u);

	/* Nothing in visibility any more. */
	if (!hasTx && !hasRx)
		return hadTx || hadRx ? DISABLE_TX_RX : KEEP_TRANSCEIVER_STATE;

	/* A direction just became available. */
	if ((hasTx && !hadTx) || (hasRx && !hadRx)) {
		if (hasTx && hasRx)
			return ENABLE_TX_AND_RX;
		return hasTx ? ENABLE_TX_ONLY : ENABLE_RX_ONLY;
	}

	/* A direction was lost while the other one remains. */
	if (hadTx && !hasTx)
		return ENABLE_RX_ONLY;
	if (hadRx && !hasRx)
		return ENABLE_TX_ONLY;

	/* Same directions: report which one changed generation. */
	{
		bool txChanged = (uplink != prevUplink);
		bool rxChanged = (downlink != prevDownlink);

		if (txChanged && rxChanged)
			return CHANGE_TX_PLUS_RX_MOD;
		if (txChanged)
			return CHANGE_TX_MOD;
		if (rxChanged)
			return CHANGE_RX_MOD;
	}

	return KEEP_TRANSCEIVER_STATE;
}
