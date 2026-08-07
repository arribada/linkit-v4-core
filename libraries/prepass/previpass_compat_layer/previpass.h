/**
 * @file  previpass.h
 * @brief PREPAS v3.4 API on top of the Kineis PREPAS v4.0 engine.
 *
 * Drop-in replacement for the v3.4 previpass.h: application code written
 * against the v3.4 API compiles unchanged. The v4.0 engine files (prepas.c,
 * prepas.h, utils.c, utils.h) are used as-is and must be compiled with this
 * layer; the v3.4 previpass.c must NOT be compiled.
 *
 * Differences from v3.4 an application must know about:
 *  - Satellite statuses follow the current constellation model: one
 *    payload type (SatPayloadType_t) plus one ON/OFF mission bit per
 *    direction, exactly as broadcast in the allcast frames and in the
 *    binary AOP file. The v3.4 "format A"/"format B" encodings and the
 *    per-direction modulation levels are gone.
 *  - SatelliteNextPassPrediction_t is now a plain 12-byte struct (was an
 *    8-byte bit-field): the raw memory layout differs. See the struct
 *    documentation below.
 *  - PredictionPassConfiguration_t.computationStepSecond is ignored: the
 *    v4.0 engine is analytic and needs no time step. Three fields were
 *    appended for the parameters v4.0 added; includeCurrentPass must be
 *    set to true to keep the v3.4 behaviour.
 *  - maxPasses is still a per-satellite limit, emulated by this layer
 *    (the v4.0 engine only supports a global limit).
 *  - the uplink status keeps the v3.4 mechanism: an ordered generation
 *    matched with ">=". The downlink status is reduced to ON/OFF, since a
 *    KIM2 module only receives Kineis satellites.
 *  - The engine uses global variables: one computation at a time, do not
 *    call from concurrent tasks or interrupts.
 */

#ifndef PREVIPASS_H
#define PREVIPASS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Offset between the engine time base (seconds since 2020-01-01) and the
 * UNIX epoch (seconds since 1970-01-01).
 *
 * Pass dates returned by this layer are UNIX timestamps, as in v3.4. To
 * turn one back into a calendar date, use the engine routine:
 *   su_date_stu20_jmahms(epoch - PREVIPASS_EPOCH_2020_TO_1970, &day, ...);
 */
#define PREVIPASS_EPOCH_2020_TO_1970 1577836800L

/* ------------------------------------------------------------------------ */
/* Satellite identification and status types (unchanged from v3.4)          */
/* ------------------------------------------------------------------------ */

/** Calendar date and time (was declared in previpass_util.h in v3.4). */
struct CalendarDateTime_t {
	uint16_t year;   /**< Year   ( > 1900 ) */
	uint8_t  month;  /**< Month  ( 1..12 )  */
	uint16_t day;    /**< Day    ( 1..31 )  */
	uint8_t  hour;   /**< Hour   ( 0..23 )  */
	uint8_t  minute; /**< Minute ( 0..59 )  */
	uint8_t  second; /**< Second ( 0..59 )  */
};

/** Satellite address, 8 bits (satelliteAddress in the allcast frames). */
typedef uint8_t SatHexId_t;

/**
 * On-board payload generation, as broadcast by the constellation.
 *
 * Three bits of the 13-bit satellite status record found in the allcast
 * constellation status messages and in the binary AOP file served by the
 * Kineis API. Values are the wire values, do NOT renumber.
 *
 * This enum and the two below name values; they are STORED as uint8_t in
 * the structures, so that C++ code can initialise and assign them
 * without explicit casts.
 */
enum SatPayloadType_t {
	SAT_PAYLOAD_ARGOS_3   = 0, /**< Argos 3 payload     */
	SAT_PAYLOAD_ARGOS_NEO = 1, /**< Argos Neo payload   */
	SAT_PAYLOAD_ARGOS_4   = 2, /**< Argos 4 payload     */
	SAT_PAYLOAD_KINEIS_V1 = 3, /**< Kineis V1 payload   */
	SAT_PAYLOAD_SPARE_1   = 4, /**< Reserved            */
	SAT_PAYLOAD_SPARE_2   = 5, /**< Reserved            */
	SAT_PAYLOAD_SPARE_3   = 6, /**< Reserved            */
	SAT_PAYLOAD_SPARE_4   = 7  /**< Reserved            */
};

/**
 * Uplink status: which payload the satellite offers on the uplink, or OFF
 * when the uplink mission is inactive.
 *
 * The values are ORDERED, as in v3.4: each generation carries the message
 * types of the ones before it plus its own, so a higher value is a
 * superset. A filter therefore reads "at least this generation" and is
 * matched with ">=":
 *   SAT_UPLK_OFF           no constraint on the uplink
 *   SAT_UPLK_ON_ARGOS_3    uplink active, any generation
 *   SAT_UPLK_ON_ARGOS_4    Argos 4 or Kineis V1
 *   SAT_UPLK_ON_KINEIS_V1  Kineis V1 only, the payload that accepts every
 *                          modulation and uplink frequency
 *
 * Stored and passed as uint8_t so that C++ code can assign plain integers
 * without casts. Payload types beyond Kineis V1 map above it, so a filter
 * asking for Kineis V1 keeps matching when the constellation grows.
 */
enum SatUplinkStatus_t {
	SAT_UPLK_OFF          = 0, /**< Uplink mission inactive */
	SAT_UPLK_ON_ARGOS_3   = 1, /**< Argos 3 payload         */
	SAT_UPLK_ON_ARGOS_NEO = 2, /**< Argos Neo payload       */
	SAT_UPLK_ON_ARGOS_4   = 3, /**< Argos 4 payload         */
	SAT_UPLK_ON_KINEIS_V1 = 4  /**< Kineis V1 payload       */
};

/**
 * Uplink status naming a payload type of the wire encoding.
 * Example on an allcast satellite status record:
 *   entry.uplinkStatus = uplinkBit ? PREVIPASS_UPLINK_STATUS(payload)
 *                                  : (uint8_t)SAT_UPLK_OFF;
 */
#define PREVIPASS_UPLINK_STATUS(payloadType) ((uint8_t)((payloadType) + 1u))

/**
 * Downlink status: active or not.
 *
 * No generation here, unlike the uplink: a KIM2 module can only receive
 * Kineis satellites, so the downlink generation carries no decision. To
 * ask for a two-way pass a beacon combines the two axes, the uplink one
 * selecting the satellite:
 *   (..., SAT_DNLK_ON, SAT_UPLK_ON_KINEIS_V1, &pass)
 */
enum SatDownlinkStatus_t {
	SAT_DNLK_OFF = 0, /**< Downlink mission inactive */
	SAT_DNLK_ON  = 1  /**< Downlink mission active   */
};


/* ------------------------------------------------------------------------ */
/* Input structures (unchanged from v3.4)                                   */
/* ------------------------------------------------------------------------ */

/** Pass prediction configuration. */
struct PredictionPassConfiguration_t {
	float beaconLatitude;            /**< Geodetic latitude (deg) [-90, 90]  */
	float beaconLongitude;           /**< Geodetic longitude (deg E) [0,360] */
	struct CalendarDateTime_t start; /**< Beginning of prediction            */
	struct CalendarDateTime_t end;   /**< End of prediction                  */
	/*
	 * The engine tells two elevations apart, and so must the caller:
	 *  - the INSTANTANEOUS elevation, which decides whether the satellite
	 *    is visible at a given moment;
	 *  - the CULMINATION, the highest elevation of a pass, reached at its
	 *    middle, which says how good that pass is.
	 * minElevation bounds the first one, maxElevation and minCulmination
	 * bound the second one.
	 */

	/**
	 * Antenna mask (deg): the satellite counts as visible while its
	 * elevation is at or above this value. Sets the start, the end and
	 * therefore the duration of every pass.
	 */
	float minElevation;

	/**
	 * Upper bound on the pass culmination (deg). Rejects passes going
	 * nearly overhead. 90 disables it.
	 */
	float maxElevation;

	float minPassDurationMinute;     /**< Min pass duration (min)            */
	uint32_t maxPasses;              /**< Max number of passes PER SATELLITE */
	float timeMarginMinPer6months;   /**< Linear time margin (min/6 months)  */
	uint32_t computationStepSecond;  /**< IGNORED (v4.0 engine is analytic)  */

	/*
	 * Parameters the v4.0 engine added; they have no v3.4 equivalent and
	 * are appended here so that existing initialisers keep compiling.
	 */

	/**
	 * Lower bound on the pass culmination (deg), companion of
	 * maxElevation. Rejects grazing passes: they last long enough to be
	 * kept by minElevation, but stay so low that transmitting over them
	 * mostly wastes energy. 0 disables it.
	 */
	float minCulmination;

	/** Beacon position uncertainty (km), widens the visibility circle. */
	float beaconPositionMarginKm;

	/**
	 * Report the pass already in progress at the start date.
	 *
	 * WARNING: unlike the two fields above, zero does NOT reproduce the
	 * v3.4 behaviour. v3.4 always reported the pass in progress, so a
	 * configuration ported from v3.4 must set this field to true; leaving
	 * it zero silently drops that pass, which is the one a beacon waking
	 * up can use immediately.
	 */
	bool includeCurrentPass;
};

/** One satellite orbit bulletin (AOP entry) and status. */
struct AopSatelliteEntry_t {
	SatHexId_t satHexId;                 /**< Satellite address, 8 bits       */
	uint8_t downlinkStatus;              /**< See SatDownlinkStatus_t         */
	uint8_t uplinkStatus;                /**< See SatUplinkStatus_t           */
	struct CalendarDateTime_t bulletin;  /**< Bulletin epoch                  */
	float semiMajorAxisKm;                    /**< Semi-major axis (km)       */
	float inclinationDeg;                     /**< Orbit inclination (deg)    */
	float ascNodeLongitudeDeg;                /**< Asc. node longitude (deg)  */
	float ascNodeDriftDeg;                    /**< Asc. node drift/rev (deg)  */
	float orbitPeriodMin;                     /**< Orbit period (min)         */
	float semiMajorAxisDriftMeterPerDay;      /**< Semi-major axis drift      */
};


/* ------------------------------------------------------------------------ */
/* Output structures                                                        */
/* ------------------------------------------------------------------------ */

/**
 * One predicted satellite pass.
 *
 * Plain fixed-width fields, each read and written by name. The raw memory
 * layout differs from v3.4, which packed the same information into a
 * 64-bit bit-field: code that serialized that raw word must be adapted.
 * satHexId carries the full 8-bit satellite address; uplinkStatus names
 * the generation the satellite offered, so testing
 * pass.uplinkStatus == SAT_UPLK_ON_KINEIS_V1 answers "was this a
 * Kineis pass?".
 */
struct SatelliteNextPassPrediction_t {
	uint32_t epoch;          /**< Pass start, seconds since 1970 (UNIX) */
	uint16_t duration;       /**< Pass duration (s)                     */
	uint8_t  elevationMax;   /**< Max elevation during pass (deg)       */
	uint8_t  satHexId;       /**< Satellite address [0x00..0xFF]        */
	uint8_t  downlinkStatus; /**< See SatDownlinkStatus_t               */
	uint8_t  uplinkStatus;   /**< See SatUplinkStatus_t                 */
};

/** Sorted linked list element holding one pass prediction. */
struct SatPassLinkedListElement_t {
	struct SatelliteNextPassPrediction_t element; /**< Pass prediction   */
	struct SatPassLinkedListElement_t *next;      /**< Next, NULL at end */
};

/** Action to apply to the transceiver, from pass list processing. */
enum TransceiverAction_t {
	UNKNOWN_TRANSCEIVER_ACTION, /**< No action                    */
	ENABLE_TX_ONLY,             /**< Enable TX only               */
	ENABLE_RX_ONLY,             /**< Enable RX only               */
	ENABLE_TX_AND_RX,           /**< Enable RX after TX           */
	DISABLE_TX_RX,              /**< Disable RX/TX                */
	CHANGE_TX_MOD,              /**< TX payload mix changed       */
	CHANGE_RX_MOD,              /**< RX payload mix changed       */
	CHANGE_TX_PLUS_RX_MOD,      /**< TX and RX payload mix changed*/
	KEEP_TRANSCEIVER_STATE      /**< Keep transceiver state       */
};

/**
 * Transceiver capacity over the satellites currently in visibility.
 *
 * minUplinkStatus is the lowest generation among the satellites in sight:
 * transmitting at that level reaches all of them, which is what v3.4
 * reported. Note the consequence: one legacy satellite entering the
 * visibility circle lowers the level even when Kineis satellites are up
 * there, so a beacon that only wants the best available link should look
 * at the pass list rather than at this value.
 */
struct NextPassTransceiverCapacity_t {
	enum TransceiverAction_t trcvrActionForNextPass; /**< Transition      */
	uint8_t minUplinkStatus; /**< Lowest uplink generation in sight       */
	uint8_t downlinkStatus;  /**< SAT_DNLK_ON if any downlink is in sight */
};


/* ------------------------------------------------------------------------ */
/* v3.4 API                                                                 */
/* ------------------------------------------------------------------------ */

/** @return an AOP entry with every field zeroed and both statuses OFF. */
struct AopSatelliteEntry_t
PREVIPASS_default_aop_satellite_entry(void);

/**
 * Compute the sorted list of satellite passes over the beacon.
 *
 * The list is built in a static pool sized by PREVIPASS_MAX_PASSES (see
 * previpass.c); each call invalidates the list returned by the previous
 * call. Satellites whose statuses are both OFF are skipped.
 *
 * @param[in]  config             Pass computation configuration.
 * @param[in]  aopTable           Satellite AOP table.
 * @param[in]  nbSatsInAopTable   Number of entries in aopTable.
 * @param[out] memoryPoolOverflow Set when results were truncated.
 * @return First element of the sorted list, NULL when no pass was found.
 */
struct SatPassLinkedListElement_t *
PREVIPASS_compute_new_prediction_pass_times
(
	struct PredictionPassConfiguration_t *config,
	struct AopSatelliteEntry_t           *aopTable,
	uint8_t                               nbSatsInAopTable,
	bool                                 *memoryPoolOverflow
);

/**
 * Same as PREVIPASS_compute_new_prediction_pass_times, keeping only the
 * satellites that can serve this beacon.
 *
 * A satellite is kept when its status reaches the requested one in each
 * direction (">=", v3.4 semantics). OFF means "no constraint".
 *
 * The filtering is done by this layer: the v4.0 engine dropped it.
 */
struct SatPassLinkedListElement_t *
PREVIPASS_compute_new_prediction_pass_times_with_status
(
	struct PredictionPassConfiguration_t *config,
	struct AopSatelliteEntry_t           *aopTable,
	uint8_t                               nbSatsInAopTable,
	uint8_t                               downlinkStatus,
	uint8_t                               uplinkStatus,
	bool                                 *memoryPoolOverflow
);

/**
 * Process the pass list at the given time and return the transceiver
 * action. Stateful: the returned action is a change since the last call.
 *
 * A change is reported when the set of reachable payload types changes in
 * either direction, which is what drives a radio reconfiguration. Two
 * different satellites carrying the same payload therefore do not trigger
 * a spurious transition.
 *
 * @param[in] currentTime         Current time, seconds since 1970 (UNIX).
 * @param[in] previsionPassesList List built by this layer.
 * @return Action to apply to the transceiver.
 */
struct NextPassTransceiverCapacity_t
PREVIPASS_process_existing_sorted_passes
(
	uint32_t                           currentTime,
	struct SatPassLinkedListElement_t *previsionPassesList
);

/**
 * Get the next pass (or the pass in progress) within 24 hours of
 * config->start. config->end and config->maxPasses are ignored.
 *
 * @return true when a pass was found.
 */
bool
PREVIPASS_compute_next_pass
(
	struct PredictionPassConfiguration_t *config,
	struct AopSatelliteEntry_t           *aopTable,
	uint8_t                               nbSatsInAopTable,
	struct SatelliteNextPassPrediction_t *nextPass
);

/**
 * Same as PREVIPASS_compute_next_pass, restricted to the satellites that
 * can serve this beacon. Criteria are those of
 * PREVIPASS_compute_new_prediction_pass_times_with_status():
 *   next transmit opportunity, any satellite
 *     (cfg, aop, n, SAT_DNLK_OFF, SAT_UPLK_ON_ARGOS_3, &p)
 *   next pass able to carry a Kineis-only modulation
 *     (cfg, aop, n, SAT_DNLK_OFF, SAT_UPLK_ON_KINEIS_V1, &p)
 *   next two-way pass usable by a KIM2 module
 *     (cfg, aop, n, SAT_DNLK_ON, SAT_UPLK_ON_KINEIS_V1, &p)
 */
bool
PREVIPASS_compute_next_pass_with_status
(
	struct PredictionPassConfiguration_t *config,
	struct AopSatelliteEntry_t           *aopTable,
	uint8_t                               nbSatsInAopTable,
	uint8_t                               downlinkStatus,
	uint8_t                               uplinkStatus,
	struct SatelliteNextPassPrediction_t *nextPass
);

#ifdef __cplusplus
}
#endif

#endif /* PREVIPASS_H */
