/**
 * @file dte_protocol.hpp
 * @brief DTE protocol encoder/decoder — PARMR, PARMW, DUMPD, PASPW, etc.
 */

#pragma once

#include <ctime>
#include <cstdarg>
#include <algorithm>
#include <map>

#include "debug.hpp"

#include "base64.hpp"

#include "dte_params.hpp"
#include "dte_commands.hpp"
#include "error.hpp"
#include "bitpack.hpp"
#include "timeutils.hpp"


class PassPredictCodec {
private:
	/* 32-bit Allcast addresses, Figure 3 of the NT */
	static const uint32_t ALLCAST_UTC_TIME = 0x00000136u;
	static const uint32_t ALLCAST_AOP_MONOSAT = 0x0000026Cu;
	static const uint32_t ALLCAST_AOP_MULTISAT = 0x0000035Au;
	static const uint32_t ALLCAST_CS_SHORT = 0x00000443u;  /* 2 satellites  */
	static const uint32_t ALLCAST_CS_MEDIUM = 0x00000575u; /* 10 satellites */
	static const uint32_t ALLCAST_CS_LONG = 0x0000062Fu;   /* 17 satellites */

	/* Useful part of a message, in bits (N blocks of 99 bits). Each message is
	 * then zero padded up to the next byte boundary.
	 */
	static const unsigned int BITS_1_BLOCK = 99;
	static const unsigned int BITS_2_BLOCKS = 198;
	static const unsigned int BITS_3_BLOCKS = 297;

	static const unsigned int HEADER_BITS = 40; /* 8 transmitter + 32 address */
	static const unsigned int FCS_BITS = 16;
	static const unsigned int AOP_SAT_FIELD_BITS = 141; /* one full AOP block          */
	static const unsigned int AOP_REL_FIELD_BITS = 25;  /* 8 address + 17 delta date   */
	static const unsigned int CS_SAT_FIELD_BITS = 13;   /* 8 + 3 + 1 + 1               */
	static const unsigned int AOP_MULTISAT_NB_REL = 4;

	static const uint16_t CRC16_POLY = 0x1021u; /* X^16 + X^12 + X^5 + 1 */

	/* Scaling of the coded fields, Figures 12, 13 and 16 */
	static const uint32_t TICKS_PER_SECOND = 8;            /* 125 ms resolution        */
	static const int32_t DELTA_DATE_OFFSET_TICKS = 65600;  /* -8200 s / 0.125 s        */
	static const int32_t AN_DRIFT_OFFSET = 28000;          /* -28.000 deg / 0.001 deg  */
	static const int32_t NODAL_PERIOD_OFFSET = 844890;     /* 84.4890 min / 0.0001 min */
	static const int32_t SEMI_MAJOR_AXIS_OFFSET = 6378137; /* metres                   */

	/* Bulletin epoch T0 = 01/01/2020 00:00:00 UTC */
	static const uint16_t BULLETIN_EPOCH_YEAR = 2020;

	/** Raw AOP fields of one satellite, before scaling. */
	struct AopFields {
		uint8_t sat_hex_id;
		uint64_t date_ticks;      /* 35 bits, 125 ms steps since T0        */
		uint32_t an_longitude;    /* 19 bits, 0.001 deg                    */
		uint32_t an_drift;        /* 13 bits, 0.001 deg, offset -28.000    */
		uint32_t nodal_period;    /* 18 bits, 0.0001 min, offset 84.4890   */
		uint32_t semi_major_axis; /* 20 bits, 1 m, offset 6378137         */
		uint32_t sma_decay;       /* 10 bits, 1 dm/day, absolute value     */
		uint32_t inclination;     /* 18 bits, 0.001 deg                    */
	};

	/* ------------------------------------------------------------------
	 * Bit level helpers
	 *
	 * EXTRACT_BITS is not used here: the bulletin date field is 35 bits
	 * wide, so extraction is done through a 64-bit accessor. Bit order is
	 * the same (MSB first).
	 * ------------------------------------------------------------------ */

	static uint64_t peek_bits(const std::string &data, unsigned int pos, unsigned int nbits) {
		uint64_t value = 0;
		for (unsigned int i = 0; i < nbits; i++) {
			unsigned int p = pos + i;
			uint8_t byte = static_cast<uint8_t>(data[p >> 3]);
			value = (value << 1) | ((byte >> (7 - (p & 7))) & 1u);
		}
		return value;
	}

	static uint64_t extract_bits(const std::string &data, unsigned int &pos, unsigned int nbits) {
		uint64_t value = peek_bits(data, pos, nbits);
		pos += nbits;
		return value;
	}

	/**
	 * Frame Check Sequence, Figures 7, 14, 17 and 23: CRC16 with polynomial
	 * X^16+X^12+X^5+1, initialised to 0, no final XOR, MSB first. Computed
	 * over "Allcast address + data", the transmitting satellite address being
	 * excluded, so the range is not byte aligned.
	 */
	static uint16_t crc16_bits(const std::string &data, unsigned int pos, unsigned int nbits) {
		uint16_t crc = 0;
		for (unsigned int i = 0; i < nbits; i++) {
			unsigned int p = pos + i;
			uint8_t byte = static_cast<uint8_t>(data[p >> 3]);
			uint16_t bit = (byte >> (7 - (p & 7))) & 1u;
			uint16_t msb = (crc >> 15) & 1u;
			crc = static_cast<uint16_t>(crc << 1);
			if (msb ^ bit) crc ^= CRC16_POLY;
		}
		return crc;
	}

	/* ------------------------------------------------------------------
	 * Date helpers
	 * ------------------------------------------------------------------ */

	static bool is_leap_year(uint16_t year) { return ((year % 4) == 0 && (year % 100) != 0) || ((year % 400) == 0); }

	/** 35-bit bulletin date (125 ms steps since T0) to calendar date, UTC. */
	static void convert_bulletin_date(uint64_t date_ticks, CalendarDateTime_t &date) {
		uint64_t seconds = date_ticks / TICKS_PER_SECOND; /* truncated to the second */
		uint32_t days = static_cast<uint32_t>(seconds / 86400);
		uint32_t sec_of_day = static_cast<uint32_t>(seconds % 86400);
		uint16_t year = BULLETIN_EPOCH_YEAR;

		for (;;) {
			uint32_t days_in_year = is_leap_year(year) ? 366 : 365;
			if (days < days_in_year) break;
			days -= days_in_year;
			year++;
		}

		date.year = year;
		convert_day_of_year(date.year, days + 1, date.month, date.day);
		date.hour = static_cast<uint8_t>(sec_of_day / 3600);
		date.minute = static_cast<uint8_t>((sec_of_day % 3600) / 60);
		date.second = static_cast<uint8_t>(sec_of_day % 60);
	}

	/** true when @p candidate is strictly more recent than @p current. */
	static bool is_newer(const CalendarDateTime_t &candidate, const CalendarDateTime_t &current) {
		if (candidate.year != current.year) return candidate.year > current.year;
		if (candidate.month != current.month) return candidate.month > current.month;
		if (candidate.day != current.day) return candidate.day > current.day;
		if (candidate.hour != current.hour) return candidate.hour > current.hour;
		if (candidate.minute != current.minute) return candidate.minute > current.minute;
		return candidate.second > current.second;
	}

	/* ------------------------------------------------------------------
	 * Operating status conversion
	 *
	 * Constellation Status records carry a 3-bit payload type plus one uplink
	 * and one downlink mission bit (Figure 22). The uplink status names the
	 * payload generation, the downlink one is a plain on/off: a KIM2 can only
	 * receive Kineis satellites, so the generation carries no decision there.
	 * ------------------------------------------------------------------ */

	static uint8_t convert_ul_operating_status(uint8_t payload_type, bool operational) {
		if (!operational) return static_cast<uint8_t>(SAT_UPLK_OFF);
		return PREVIPASS_UPLINK_STATUS(payload_type);
	}

	static uint8_t convert_dl_operating_status(bool operational) {
		return operational ? static_cast<uint8_t>(SAT_DNLK_ON) : static_cast<uint8_t>(SAT_DNLK_OFF);
	}

	/* ------------------------------------------------------------------
	 * Message decoding
	 * ------------------------------------------------------------------ */

	static void extract_aop_fields(const std::string &data, unsigned int &pos, AopFields &fields) {
		fields.sat_hex_id = static_cast<uint8_t>(extract_bits(data, pos, 8));
		fields.date_ticks = extract_bits(data, pos, 35);
		fields.an_longitude = static_cast<uint32_t>(extract_bits(data, pos, 19));
		fields.an_drift = static_cast<uint32_t>(extract_bits(data, pos, 13));
		fields.nodal_period = static_cast<uint32_t>(extract_bits(data, pos, 18));
		fields.semi_major_axis = static_cast<uint32_t>(extract_bits(data, pos, 20));
		fields.sma_decay = static_cast<uint32_t>(extract_bits(data, pos, 10));
		fields.inclination = static_cast<uint32_t>(extract_bits(data, pos, 18));
	}

	/**
	 * Build one orbit params entry and insert it, keeping the most recent
	 * bulletin when the satellite is already known.
	 *
	 * @param an_longitude_millideg ascending node longitude in milli-degrees:
	 *        the raw field for a reference satellite, the value computed from
	 *        the drift for a relative one.
	 */
	static void store_orbit_params(std::map<uint8_t, AopSatelliteEntry_t> &orbit_params, uint8_t hex_id,
	                               uint64_t date_ticks, double an_longitude_millideg, const AopFields &ref) {
		AopSatelliteEntry_t aop_entry = {};

		/* 0x00 is a forbidden satellite address, it marks zero padding */
		if (hex_id == 0x00) return;

		aop_entry.satHexId = hex_id;
		convert_bulletin_date(date_ticks, aop_entry.bulletin);
		aop_entry.ascNodeLongitudeDeg = static_cast<float>(an_longitude_millideg / 1000.0);
		aop_entry.ascNodeDriftDeg = static_cast<float>((static_cast<double>(ref.an_drift) - AN_DRIFT_OFFSET) / 1000.0);
		aop_entry.orbitPeriodMin =
		    static_cast<float>((static_cast<double>(ref.nodal_period) + NODAL_PERIOD_OFFSET) / 10000.0);
		aop_entry.semiMajorAxisKm =
		    static_cast<float>((static_cast<double>(ref.semi_major_axis) + SEMI_MAJOR_AXIS_OFFSET) / 1000.0);
		/* The coded field is an absolute decay value ("75 means -75 dm/day",
		 * Figure 12), the entry carries the signed, negative drift.
		 */
		aop_entry.semiMajorAxisDriftMeterPerDay =
		    (ref.sma_decay == 0) ? 0.0f : static_cast<float>(-0.1 * static_cast<double>(ref.sma_decay));
		aop_entry.inclinationDeg = static_cast<float>(static_cast<double>(ref.inclination) / 1000.0);

		DEBUG_TRACE("allcast_sat_orbit_params_decode: hex_id=%02x dd/mm/yy=%u/%u/%u hh:mm:ss=%u:%u:%u "
		            "a=%f i=%f an=%f",
		            aop_entry.satHexId, aop_entry.bulletin.day, aop_entry.bulletin.month, aop_entry.bulletin.year,
		            aop_entry.bulletin.hour, aop_entry.bulletin.minute, aop_entry.bulletin.second,
		            (double)aop_entry.semiMajorAxisKm, (double)aop_entry.inclinationDeg,
		            (double)aop_entry.ascNodeLongitudeDeg);

		std::map<uint8_t, AopSatelliteEntry_t>::iterator it = orbit_params.find(hex_id);
		if (it != orbit_params.end() && !is_newer(aop_entry.bulletin, it->second.bulletin))
			return; /* an equal or more recent bulletin is already held */

		orbit_params[hex_id] = aop_entry;
	}

	/**
	 * AOP MonoSat and AOP MultiSat messages. A MultiSat message holds the full
	 * orbit params of a reference satellite plus, for up to 4 relative
	 * satellites of the same orbital plane, an address and a delta date: every
	 * other parameter is shared with the reference, only the ascending node
	 * longitude has to be propagated (Annex A).
	 */
	static void allcast_sat_orbit_params_decode(const std::string &data, unsigned int &pos, bool multisat,
	                                            std::map<uint8_t, AopSatelliteEntry_t> &orbit_params) {
		AopFields ref;

		extract_aop_fields(data, pos, ref);
		store_orbit_params(orbit_params, ref.sat_hex_id, ref.date_ticks, static_cast<double>(ref.an_longitude), ref);

		if (!multisat) return;

		/* Ascending node longitude drift, in milli-degrees per 125 ms step */
		double drift_coefficient = (((static_cast<double>(ref.an_drift) - AN_DRIFT_OFFSET) * 0.001)
		                            / ((static_cast<double>(ref.nodal_period) + NODAL_PERIOD_OFFSET) * 0.0001))
		                           / 0.001 / 60.0 * 0.125;

		for (unsigned int i = 0; i < AOP_MULTISAT_NB_REL; i++) {
			uint8_t hex_id = static_cast<uint8_t>(extract_bits(data, pos, 8));
			uint32_t delta_date = static_cast<uint32_t>(extract_bits(data, pos, 17));

			/* A null address is zero padding: nothing usable afterwards */
			if (hex_id == 0x00) break;

			int32_t delta_ticks = static_cast<int32_t>(delta_date) - DELTA_DATE_OFFSET_TICKS;
			int64_t date_ticks = static_cast<int64_t>(ref.date_ticks) + delta_ticks;
			if (date_ticks < 0) date_ticks = 0;

			double an_longitude =
			    static_cast<double>(ref.an_longitude) + drift_coefficient * static_cast<double>(delta_ticks);
			while (an_longitude < 0.0)
				an_longitude += 360000.0;
			while (an_longitude >= 360000.0)
				an_longitude -= 360000.0;

			store_orbit_params(orbit_params, hex_id, static_cast<uint64_t>(date_ticks), an_longitude, ref);
		}
	}

	/**
	 * Constellation Status messages, in their 2, 10 or 17 satellites version.
	 * The counter, index and total number of messages let a receiver tell a new
	 * status set from an incomplete one; they are decoded for tracing only.
	 */
	static void allcast_constellation_status_decode(const std::string &data, unsigned int &pos,
	                                                unsigned int num_satellites,
	                                                std::map<uint8_t, AopSatelliteEntry_t> &constellation_params,
	                                                AllcastStatusTracking *tracking) {
		uint8_t counter = static_cast<uint8_t>(extract_bits(data, pos, 6) + 1);
		uint8_t index = static_cast<uint8_t>(extract_bits(data, pos, 3) + 1);
		uint8_t total = static_cast<uint8_t>(extract_bits(data, pos, 3) + 1);

		DEBUG_TRACE("allcast_constellation_status_decode: counter=%u index=%u/%u num_satellites=%u", counter, index,
		            total, num_satellites);

		if (tracking != nullptr) {
			/* A new counter value means a new status set: the Kineis service
			 * centre increments it on every constellation change. The previous
			 * set is stale and must be DROPPED, not merged with the new one —
			 * a satellite that has been decommissioned simply stops being
			 * listed, and the NT states that its absence from the current CS
			 * messages is how a device learns it is no longer operational.
			 * Keeping its entry would leave the merged table waiting forever
			 * for an orbit bulletin that will never be broadcast again.
			 */
			if (tracking->counter != counter) {
				if (tracking->counter != 0) {
					DEBUG_INFO("allcast_constellation_status_decode: constellation "
					           "status set changed (%u -> %u), dropping the previous "
					           "%u satellite status entries",
					           tracking->counter, counter, (unsigned int)constellation_params.size());
					constellation_params.clear();
				}
				tracking->counter = counter;
				tracking->index_mask = 0;
			}
			tracking->total = total;
			if (index >= 1 && index <= 8) tracking->index_mask |= static_cast<uint8_t>(1u << (index - 1));
		}

		for (unsigned int i = 0; i < num_satellites; i++) {
			AopSatelliteEntry_t aop_entry = {};
			uint8_t hex_id = static_cast<uint8_t>(extract_bits(data, pos, 8));
			uint8_t payload_type = static_cast<uint8_t>(extract_bits(data, pos, 3));
			bool ul_operational = extract_bits(data, pos, 1) != 0;
			bool dl_operational = extract_bits(data, pos, 1) != 0;

			/* A null address is zero padding up to the FCS */
			if (hex_id == 0x00) break;

			DEBUG_TRACE("allcast_constellation_status_decode: sat=%u hex_id=%02x payload=%01x "
			            "dl_status=%u ul_status=%u",
			            i, hex_id, payload_type, (unsigned)dl_operational, (unsigned)ul_operational);

			aop_entry.satHexId = hex_id;
			aop_entry.uplinkStatus = convert_ul_operating_status(payload_type, ul_operational);
			aop_entry.downlinkStatus = convert_dl_operating_status(dl_operational);

			uint8_t key = aop_entry.satHexId;
			constellation_params[key] = aop_entry;
		}
	}

	/**
	 * Decode one Allcast message. @p pos is advanced by the whole message,
	 * zero padding included, so that a caller can walk a stream of concatenated
	 * messages.
	 */
	static bool is_known_allcast_address(uint32_t address) {
		return address == ALLCAST_AOP_MONOSAT || address == ALLCAST_AOP_MULTISAT || address == ALLCAST_CS_SHORT
		       || address == ALLCAST_CS_MEDIUM || address == ALLCAST_CS_LONG || address == ALLCAST_UTC_TIME;
	}

	static void allcast_packet_decode(const std::string &data, unsigned int &pos,
	                                  std::map<uint8_t, AopSatelliteEntry_t> &orbit_params,
	                                  std::map<uint8_t, AopSatelliteEntry_t> &constellation_status,
	                                  AllcastStatusTracking *tracking) {
		unsigned int start = pos;
		unsigned int available = static_cast<unsigned int>(8 * data.length()) - start;
		unsigned int address_offset;
		unsigned int useful_bits;
		unsigned int message_bits;
		unsigned int num_satellites = 0;
		uint32_t allcast_address;
		uint16_t fcs, computed_fcs;

		if (available < 32) {
			DEBUG_ERROR("allcast_packet_decode: truncated message");
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}

		/* The useful part of a message starts with the 8-bit address of the
		 * transmitting satellite (0x00 when served by the API). Some receivers
		 * hand over the frame without it, so both layouts are accepted: the
		 * known 32-bit Allcast addresses are all below 0x1000, which makes the
		 * detection unambiguous.
		 */
		allcast_address = static_cast<uint32_t>(peek_bits(data, start, 32));
		if (is_known_allcast_address(allcast_address)) {
			address_offset = 0;
		} else if (available >= HEADER_BITS) {
			address_offset = 8;
			allcast_address = static_cast<uint32_t>(peek_bits(data, start + 8, 32));
		} else {
			DEBUG_ERROR("allcast_packet_decode: truncated message");
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}

		switch (allcast_address) {
		case ALLCAST_AOP_MONOSAT: useful_bits = BITS_2_BLOCKS; break;
		case ALLCAST_AOP_MULTISAT: useful_bits = BITS_3_BLOCKS; break;
		case ALLCAST_CS_SHORT:
			useful_bits = BITS_1_BLOCK;
			num_satellites = 2;
			break;
		case ALLCAST_CS_MEDIUM:
			useful_bits = BITS_2_BLOCKS;
			num_satellites = 10;
			break;
		case ALLCAST_CS_LONG:
			useful_bits = BITS_3_BLOCKS;
			num_satellites = 17;
			break;
		case ALLCAST_UTC_TIME: useful_bits = BITS_1_BLOCK; break;
		default:
			DEBUG_ERROR("allcast_packet_decode: unrecognised allcast address (%08x)", allcast_address);
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}

		/* Every message spans a whole number of bytes: 99, 198 or 297 useful
		 * bits padded with 5, 2 or 7 zero bits. A frame handed over without
		 * the transmitting satellite address is 8 bits shorter.
		 */
		useful_bits = useful_bits - 8 + address_offset;
		message_bits = 8 * ((useful_bits + 7) / 8);
		if (available < message_bits) {
			DEBUG_ERROR("allcast_packet_decode: truncated message (%u bits available, %u needed)", available,
			            message_bits);
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}

		fcs = static_cast<uint16_t>(peek_bits(data, start + useful_bits - FCS_BITS, FCS_BITS));
		computed_fcs = crc16_bits(data, start + address_offset, useful_bits - FCS_BITS - address_offset);
		if (fcs != computed_fcs) {
			DEBUG_ERROR("allcast_packet_decode: bad FCS (%04x, expected %04x)", fcs, computed_fcs);
			pos = start + message_bits; /* frame boundary is known, skip it */
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}

		pos = start + address_offset + 32;
		switch (allcast_address) {
		case ALLCAST_AOP_MONOSAT: allcast_sat_orbit_params_decode(data, pos, false, orbit_params); break;
		case ALLCAST_AOP_MULTISAT: allcast_sat_orbit_params_decode(data, pos, true, orbit_params); break;
		case ALLCAST_UTC_TIME: DEBUG_TRACE("allcast_packet_decode: UTC time message skipped"); break;
		default: allcast_constellation_status_decode(data, pos, num_satellites, constellation_status, tracking); break;
		}

		pos = start + message_bits;
	}

	/**
	 * Merge the two maps into the output table. A record is emitted for every
	 * satellite of the constellation status map; its orbit params are copied
	 * when they are known and the satellite is operational, otherwise the
	 * bulletin year is left at 0 to flag a missing AOP.
	 */
	static void merge(const std::map<uint8_t, AopSatelliteEntry_t> &orbit_params,
	                  const std::map<uint8_t, AopSatelliteEntry_t> &constellation_status,
	                  BasePassPredict &pass_predict) {
		unsigned int num_records = 0;
		unsigned int with_aop = 0;
		unsigned int not_operational = 0;

		pass_predict.num_records = 0;

		/* No per-satellite tracing here on purpose: merge() runs on EVERY
		 * received packet, so one line per satellite means ~25 log lines per
		 * second during a pass. On a flash-backed logger that blocks the main
		 * loop for several seconds, the UART RX buffer overflows, allcast lines
		 * get truncated and merged, and the module's answers are lost. One
		 * summary line at the end instead.
		 */
		for (const auto &it : constellation_status) {
			if (num_records >= MAX_AOP_SATELLITE_ENTRIES) {
				DEBUG_WARN("PassPredictCodec::merge: discard entry hex_id=%02x as full", it.second.satHexId);
				continue;
			}

			pass_predict.records[num_records] = AopSatelliteEntry_t();

			/* Don't expect an AOP unless either downlink or uplink is operational */
			if (it.second.downlinkStatus || it.second.uplinkStatus) {
				std::map<uint8_t, AopSatelliteEntry_t>::const_iterator aop = orbit_params.find(it.first);

				if (aop != orbit_params.end()) {
					pass_predict.records[num_records] = aop->second;
					with_aop++;
				} else {
					pass_predict.records[num_records].bulletin.year = 0;
				}
			} else {
				pass_predict.records[num_records].bulletin.year = 0;
				not_operational++;
			}

			pass_predict.records[num_records].satHexId = it.second.satHexId;
			pass_predict.records[num_records].downlinkStatus = it.second.downlinkStatus;
			pass_predict.records[num_records].uplinkStatus = it.second.uplinkStatus;
			num_records++;
		}

		pass_predict.num_records = num_records;

		DEBUG_TRACE("PassPredictCodec::merge: %u declared, %u with AOP, %u out of service", num_records, with_aop,
		            not_operational);
	}

public:
	// This decode variant processes a single packet at a time and is supplied a map
	// of existing orbit params and constellation status
	static void decode(std::map<uint8_t, AopSatelliteEntry_t> &orbit_params,
	                   std::map<uint8_t, AopSatelliteEntry_t> &constellation_status, std::string const &data,
	                   BasePassPredict &pass_predict) {
		decode(orbit_params, constellation_status, nullptr, data, pass_predict);
	}

	// Same, keeping track of the constellation status set: tracking.is_complete()
	// tells whether every CS message of the current set has been received, which
	// is what says the constellation picture is whole.
	static void decode(std::map<uint8_t, AopSatelliteEntry_t> &orbit_params,
	                   std::map<uint8_t, AopSatelliteEntry_t> &constellation_status, AllcastStatusTracking *tracking,
	                   std::string const &data, BasePassPredict &pass_predict) {
		// The two maps are the caller's persistent state: an AOP message only
		// updates orbit_params, a constellation status message only updates
		// constellation_status, so both can be refreshed independently as the
		// KIM2 receives them, in any order.
		try {
			unsigned int base_pos = 0;
			allcast_packet_decode(data, base_pos, orbit_params, constellation_status, tracking);
		} catch (...) {
			// Ignore any errors decoding the packet and just move onto the next
		}

		merge(orbit_params, constellation_status, pass_predict);
	}

	// This decode variant walks a buffer of concatenated messages, as served by
	// the Kineis API. Message boundaries are known from the Allcast address, so
	// a corrupted message is skipped instead of aborting the whole buffer.
	static void decode(const std::string &data, BasePassPredict &pass_predict,
	                   AllcastStatusTracking *tracking = nullptr) {
		unsigned int base_pos = 0;
		std::map<uint8_t, AopSatelliteEntry_t> orbit_params;
		std::map<uint8_t, AopSatelliteEntry_t> constellation_status;

		while (base_pos + HEADER_BITS <= (8 * data.length())) {
			unsigned int previous_pos = base_pos;

			try {
				allcast_packet_decode(data, base_pos, orbit_params, constellation_status, tracking);
			} catch (...) {
				// Unknown address or bad FCS: stop unless the frame length was
				// known, in which case allcast_packet_decode already moved past it
				if (base_pos == previous_pos) break;
			}
		}

		merge(orbit_params, constellation_status, pass_predict);
	}
};


class DTEDecoder;

class DTEEncoder {
private:
	static inline void string_sprintf(std::string &str, const char *fmt, ...) {
		// Provides a way to easily sprintf append to a std::string
		char buff[256];
		va_list args;
		va_start(args, fmt);
		int written = vsnprintf(buff, sizeof(buff), fmt, args);
		va_end(args);
		if (written < 0 || written >= static_cast<int>(sizeof(buff))) throw DTE_PROTOCOL_MESSAGE_TOO_LARGE;
		str.append(buff, written);
	}

protected:
	static inline void encode(std::string &output, const BaseGNSSFixMode &value) {
		encode(output, (unsigned int &)value);
	}
	static inline void encode(std::string &output, const BaseGNSSDynModel &value) {
		encode(output, (unsigned int &)value);
	}
	static inline void encode(std::string &output, const BaseLEDMode &value) { encode(output, (unsigned int &)value); }
	static inline void encode(std::string &output, const BaseZoneType &value) { encode(output, (unsigned int &)value); }
	static inline void encode(std::string &output, const BaseDebugMode &value) {
		encode(output, (unsigned int &)value);
	}
	static inline void encode(std::string &output, const BasePressureSensorLoggingMode &value) {
		encode(output, (unsigned int &)value);
	}
	static inline void encode(std::string &output, const BasePressureSensorFullScale &value) {
		encode(output, (unsigned int &)value);
	}
	static inline void encode(std::string &output, const std::time_t &value) {
		char buff[256];
		auto time = std::gmtime(&value);
		if (!time) {
			output.append("00/00/0000 00:00:00");
			return;
		}
		int written = std::strftime(buff, sizeof(buff), "%d/%m/%Y %H:%M:%S", time);

		if (written == 0) throw DTE_PROTOCOL_MESSAGE_TOO_LARGE;

		output.append(buff, written);
	}
	static inline void encode(std::string &output, const unsigned int &value) { string_sprintf(output, "%u", value); }
	static inline void encode(std::string &output, const unsigned int &value, bool hex) {
		if (hex)
			string_sprintf(output, "%X", value);
		else
			string_sprintf(output, "%u", value);
	}
	static inline void encode(std::string &output, const bool &value) {
		encode(output, static_cast<unsigned int>(value));
	}
	static inline void encode(std::string &output, const int &value) { string_sprintf(output, "%d", value); }
	static inline void encode(std::string &output, const BaseRawData &value) {
		std::string s;
		if (value.length == 0) {
			s = websocketpp::base64_encode(value.str);
		} else {
			s = websocketpp::base64_encode((unsigned char const *)value.ptr, value.length);
		}
		// Don't payload size to exceed max permitted length
		if (s.length() > BASE_MAX_PAYLOAD_LENGTH) throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		output.append(s);
	}
	static inline void encode(std::string &output, const double &value) { string_sprintf(output, "%g", value); }
	static inline void encode(std::string &output, const std::string &value) { output.append(value); }
	static inline void encode(std::string &output, const ParamID &value) { output.append(param_map[(int)value].key); }
	static inline void encode(std::string &output, const BaseDepthPile &value) {
		unsigned int depth_pile = 0;
		if (value == BaseDepthPile::DEPTH_PILE_1)
			depth_pile = 1;
		else if (value == BaseDepthPile::DEPTH_PILE_2)
			depth_pile = 2;
		else if (value == BaseDepthPile::DEPTH_PILE_3)
			depth_pile = 3;
		else if (value == BaseDepthPile::DEPTH_PILE_4)
			depth_pile = 4;
		else if (value == BaseDepthPile::DEPTH_PILE_8)
			depth_pile = 8;
		else if (value == BaseDepthPile::DEPTH_PILE_12)
			depth_pile = 9;
		else if (value == BaseDepthPile::DEPTH_PILE_16)
			depth_pile = 10;
		else if (value == BaseDepthPile::DEPTH_PILE_20)
			depth_pile = 11;
		else if (value == BaseDepthPile::DEPTH_PILE_24)
			depth_pile = 12;
		encode(output, depth_pile);
	}
	static inline void encode(std::string &output, const BaseSensorEnableTxMode &value) {
		encode(output, (unsigned int)value);
	}
	static inline void encode_acquisition_period(std::string &output, unsigned int &value) {
		unsigned int x;
		switch (value) {
		case 0: x = 0; break;
		case 10 * 60: x = 1; break;     // 10 min
		case 15 * 60: x = 2; break;     // 15 min
		case 30 * 60: x = 3; break;     // 30 min
		case 60 * 60: x = 4; break;     // 1 hour
		case 120 * 60: x = 5; break;    // 2 hours
		case 180 * 60: x = 6; break;    // 3 hours
		case 240 * 60: x = 7; break;    // 4 hours
		case 360 * 60: x = 8; break;    // 6 hours
		case 720 * 60: x = 9; break;    // 12 hours
		case 1440 * 60: x = 10; break;  // 24 hours
		case 1 * 60: x = 11; break;     // 1 min
		case 2 * 60: x = 12; break;     // 2 min
		case 5 * 60: x = 13; break;     // 5 min
		case 20 * 60: x = 14; break;    // 20 min
		case 45 * 60: x = 15; break;    // 45 min
		default: throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
		encode(output, x);
	}
	static inline void encode_frequency(std::string &output, double &value) {
		unsigned int x = (value * ARGOS_FREQUENCY_MULT) - ARGOS_FREQUENCY_OFFSET;
		encode(output, x);
	}
	static inline void encode(std::string &output, const BaseArgosMode &value) {
		encode(output, (unsigned int &)value);
	}
	static inline void encode(std::string &output, const BaseArgosPower &value) {
		encode(output, (unsigned int &)value);
	}
	static inline void encode(std::string &output, const BaseArgosModulation &value) {
		encode(output, (unsigned int &)value);
	}
	static inline void encode(std::string &output, const BaseUnderwaterDetectSource &value) {
		encode(output, (unsigned int &)value);
	}
	static void validate(const BaseMap &arg_map, const std::string &value) {
		if (value.length() > BASE_TEXT_MAX_LENGTH) {
			DEBUG_ERROR("parameter \"%s\" string length %u is out of bounds", arg_map.name.c_str(), value.length());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
		if (!arg_map.permitted_values.empty()
		    && std::find_if(arg_map.permitted_values.begin(), arg_map.permitted_values.end(),
		                    [value](const BaseConstraint x) { return std::get<std::string>(x) == value; })
		           == arg_map.permitted_values.end()) {
			DEBUG_ERROR("parameter \"%s\" not in permitted list", arg_map.name.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static void validate(const BaseMap &arg_map, const double &value) {
		const auto min_value = std::get<double>(arg_map.min_value);
		const auto max_value = std::get<double>(arg_map.max_value);
		if ((min_value != 0 || max_value != 0) && (value < min_value || value > max_value)) {
			DEBUG_ERROR("parameter \"%s\" value out of min/max range", arg_map.name.c_str(), value, min_value,
			            max_value);
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
		if (!arg_map.permitted_values.empty()
		    && std::find_if(arg_map.permitted_values.begin(), arg_map.permitted_values.end(),
		                    [value](const BaseConstraint x) { return std::get<double>(x) == value; })
		           == arg_map.permitted_values.end()) {
			DEBUG_ERROR("parameter \"%s\" not in permitted list", arg_map.name.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static void validate(const BaseMap &arg_map, const unsigned int &value) {
		const auto min_value = std::get<unsigned int>(arg_map.min_value);
		const auto max_value = std::get<unsigned int>(arg_map.max_value);
		if ((min_value != 0 || max_value != 0) && (value < min_value || value > max_value)) {
			DEBUG_ERROR("parameter \"%s\" value out of min/max range", arg_map.name.c_str(), value, min_value,
			            max_value);
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
		if (!arg_map.permitted_values.empty()
		    && std::find_if(arg_map.permitted_values.begin(), arg_map.permitted_values.end(),
		                    [value](const BaseConstraint x) { return std::get<unsigned int>(x) == value; })
		           == arg_map.permitted_values.end()) {
			DEBUG_ERROR("parameter \"%s\" not in permitted list", arg_map.name.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static void validate(const BaseMap &arg_map, const int &value) {
		const auto min_value = std::get<int>(arg_map.min_value);
		const auto max_value = std::get<int>(arg_map.max_value);
		if ((min_value != 0 || max_value != 0) && (value < min_value || value > max_value)) {
			DEBUG_ERROR("parameter \"%s\" value out of min/max range", arg_map.name.c_str(), value, min_value,
			            max_value);
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
		if (!arg_map.permitted_values.empty()
		    && std::find_if(arg_map.permitted_values.begin(), arg_map.permitted_values.end(),
		                    [value](const BaseConstraint x) { return std::get<int>(x) == value; })
		           == arg_map.permitted_values.end()) {
			DEBUG_ERROR("parameter \"%s\" not in permitted list", arg_map.name.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	/// @brief Validate a boolean parameter (no range check needed).
	static void validate(const BaseMap &, const bool &) {}
	static void validate(const BaseMap &, const std::time_t &) {}
	static void validate(const BaseMap &, const BaseRawData &) {}
	// Generic enum validator: checks value against permitted_values if defined
	template <typename EnumT> static void validate_enum(const BaseMap &arg_map, const EnumT &value) {
		if (!arg_map.permitted_values.empty()) {
			unsigned int raw = static_cast<unsigned int>(value);
			if (std::find_if(arg_map.permitted_values.begin(), arg_map.permitted_values.end(),
			                 [raw](const BaseConstraint x) { return std::get<unsigned int>(x) == raw; })
			    == arg_map.permitted_values.end()) {
				DEBUG_ERROR("parameter \"%s\" enum value %u not in permitted list", arg_map.name.c_str(), raw);
				throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
			}
		}
	}
	static void validate(const BaseMap &m, const BaseGNSSFixMode &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseGNSSDynModel &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseDepthPile &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseArgosMode &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseSensorEnableTxMode &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseUnderwaterDetectSource &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseArgosPower &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseLEDMode &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseZoneType &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseArgosModulation &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BaseDebugMode &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BasePressureSensorLoggingMode &v) { validate_enum(m, v); }
	static void validate(const BaseMap &m, const BasePressureSensorFullScale &v) { validate_enum(m, v); }

public:
	// FIXME: Using C variadic args with non-POD types (std::string, BaseRawData) is
	// undefined behavior in C++. Works on GCC 10.3 ARM but may break with compiler
	// updates. Should be refactored to use variadic templates or typed overloads.
	static std::string encode(DTECommand command, ...) {
		unsigned int error_code = 0;
		std::string buffer;
		std::string payload;
		unsigned int command_index = (unsigned int)command & RESP_CMD_BASE ? ((unsigned int)command & ~RESP_CMD_BASE)
		                                                                         + (unsigned int)DTECommand::__NUM_REQ
		                                                                   : (unsigned int)command;
		const std::string &command_name = command_map[command_index].name;
		const std::vector<BaseMap> &command_args = command_map[command_index].prototype;
		unsigned int expected_args = command_args.size();

		DEBUG_TRACE("command = %u expected_args = %u", (unsigned int)command, expected_args);

		va_list args;
		va_start(args, command);

		if (((unsigned int)command & RESP_CMD_BASE)) {
			error_code = va_arg(args, unsigned int);
			DEBUG_TRACE("error_code %u", error_code);
			if (error_code > 0) {
				DEBUG_TRACE("abort error_code %u", error_code);
				encode(payload, error_code);
				expected_args = 0;
			}
		}

		// Ignore additional arguments if an error was indicated or expected number of args does match
		for (unsigned int arg_index = 0; arg_index < expected_args; arg_index++) {
			DEBUG_TRACE("arg_index = %u command = %u encoding = %u", arg_index, (unsigned int)command,
			            (unsigned int)command_args[arg_index].encoding);
			// Add separator for next argument to follow
			if (arg_index > 0) payload.append(",");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"

			switch (command_args[arg_index].encoding) {
			case BaseEncoding::DECIMAL: {
				int arg = va_arg(args, int);
				validate(command_args[arg_index], arg);
				encode(payload, arg);
				break;
			}
			case BaseEncoding::FLOAT: {
				double arg = va_arg(args, double);
				validate(command_args[arg_index], arg);
				encode(payload, arg);
				break;
			}
			case BaseEncoding::UINT: {
				unsigned int arg = va_arg(args, unsigned int);
				validate(command_args[arg_index], arg);
				encode(payload, arg, false);
				break;
			}
			case BaseEncoding::HEXADECIMAL: {
				unsigned int arg = va_arg(args, unsigned int);
				validate(command_args[arg_index], arg);
				encode(payload, arg, true);
				break;
			}
			case BaseEncoding::BOOLEAN: {
				bool arg = va_arg(args, unsigned int);
				encode(payload, arg);
				break;
			}
			case BaseEncoding::BASE64: encode(payload, va_arg(args, BaseRawData)); break;
			case BaseEncoding::TEXT: {
				DEBUG_TRACE("Encoding TEXT....");
				std::string arg = va_arg(args, std::string);
				DEBUG_TRACE("Checking %s....", arg.c_str());
				validate(command_args[arg_index], arg);
				encode(payload, arg);
				break;
			}
			default: DEBUG_ERROR("parameter type not permitted"); throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
			}

#pragma GCC diagnostic pop
		}
		va_end(args);

		// Construct header depending on case of request, response with error or response without error
		if ((unsigned int)command & RESP_CMD_BASE) {
			if (error_code)
				buffer.append("$N;");
			else
				buffer.append("$O;");
		} else {
			buffer.append("$");
		}

		// Sanity check payload length
		if (payload.size() > BASE_MAX_PAYLOAD_LENGTH) {
			DEBUG_ERROR("DTE_PROTOCOL_MESSAGE_TOO_LARGE");
			throw DTE_PROTOCOL_MESSAGE_TOO_LARGE;
		}

		// Append command, separator, payload and terminate
		buffer.append(command_name);
		buffer.append("#");
		string_sprintf(buffer, "%03X", payload.size());
		buffer.append(";");
		buffer.append(payload);
		buffer.append("\r");

		return buffer;
	}

	static std::string encode(DTECommand command, const BaseRawData &raw_data) {
		std::string buffer;
		std::string payload;
		unsigned int command_index = (unsigned int)command & RESP_CMD_BASE ? ((unsigned int)command & ~RESP_CMD_BASE)
		                                                                         + (unsigned int)DTECommand::__NUM_REQ
		                                                                   : (unsigned int)command;
		const std::string &command_name = command_map[command_index].name;

		encode(payload, raw_data);

		if ((unsigned int)command & RESP_CMD_BASE) {
			buffer.append("$O;");
		} else {
			buffer.append("$");
		}

		if (payload.size() > BASE_MAX_PAYLOAD_LENGTH) {
			DEBUG_ERROR("DTE_PROTOCOL_MESSAGE_TOO_LARGE");
			throw DTE_PROTOCOL_MESSAGE_TOO_LARGE;
		}

		buffer.append(command_name);
		buffer.append("#");
		string_sprintf(buffer, "%03X", payload.size());
		buffer.append(";");
		buffer.append(payload);
		buffer.append("\r");

		return buffer;
	}

	static std::string encode(DTECommand command, std::vector<ParamID> &params) {
		std::string buffer;
		std::string payload;
		const std::string &command_name = command_map[(unsigned int)command & ~RESP_CMD_BASE].name;
		unsigned int expected_args = params.size();

		// Ignore additional arguments if an error was indicated or expected number of args does match
		for (unsigned int arg_index = 0; arg_index < expected_args; arg_index++) {
			// Add separator for next argument to follow
			if (arg_index > 0) payload.append(",");
			encode(payload, params[arg_index]);
		}

		// Construct header depending on case of request, response with error or response without error
		if ((unsigned int)command & RESP_CMD_BASE) {
			buffer.append("$O;");
		} else {
			buffer.append("$");
		}

		// Sanity check payload length
		if (payload.size() > BASE_MAX_PAYLOAD_LENGTH) {
			DEBUG_ERROR("DTE_PROTOCOL_MESSAGE_TOO_LARGE");
			throw DTE_PROTOCOL_MESSAGE_TOO_LARGE;
		}

		// Append command, separator, payload and terminate
		buffer.append(command_name);
		buffer.append("#");
		string_sprintf(buffer, "%03X", payload.size());
		buffer.append(";");
		buffer.append(payload);
		buffer.append("\r");

		return buffer;
	}

	static std::string encode(DTECommand command, std::vector<ParamValue> &param_values) {
		std::string buffer;
		std::string payload;
		const std::string &command_name = command_map[(unsigned int)command & ~RESP_CMD_BASE].name;
		unsigned int expected_args = param_values.size();

		// Ignore additional arguments if an error was indicated or expected number of args does match
		for (unsigned int arg_index = 0; arg_index < expected_args; arg_index++) {
			const BaseMap &map = param_map[(unsigned int)param_values[arg_index].param];
			// Add separator for next argument to follow
			if (arg_index > 0) payload.append(",");
			encode(payload, param_values[arg_index].param);
			payload.append("=");
			//std::cout << "arg_index:" << arg_index << " enc: " << (unsigned)param_map[(unsigned int)param_values[arg_index].param].encoding << " type:" << param_values[arg_index].value.index() << "\n";
			if (param_map[(unsigned int)param_values[arg_index].param].encoding == BaseEncoding::HEXADECIMAL) {
				unsigned int value = std::get<unsigned int>(param_values[arg_index].value);
				validate(map, value);
				encode(payload, value, true);
			} else if (param_map[(unsigned int)param_values[arg_index].param].encoding == BaseEncoding::ARGOSFREQ) {
				double value = std::get<double>(param_values[arg_index].value);
				validate(map, value);
				encode_frequency(payload, value);
			} else if (param_map[(unsigned int)param_values[arg_index].param].encoding == BaseEncoding::AQPERIOD) {
				unsigned int value = std::get<unsigned int>(param_values[arg_index].value);
				encode_acquisition_period(payload, value);
			} else {
				std::visit(
				    [&map, &payload](auto &&arg) {
					    validate(map, arg);
					    encode(payload, arg);
				    },
				    param_values[arg_index].value);
			}
		}

		// Construct header depending on case of request, response with error or response without error
		if ((unsigned int)command & RESP_CMD_BASE) {
			buffer.append("$O;");
		} else {
			buffer.append("$");
		}

		// Sanity check payload length
		if (payload.size() > BASE_MAX_PAYLOAD_LENGTH) {
			DEBUG_ERROR("DTE_PROTOCOL_MESSAGE_TOO_LARGE");
			throw DTE_PROTOCOL_MESSAGE_TOO_LARGE;
		}

		// Append command, separator, payload and terminate
		buffer.append(command_name);
		buffer.append("#");
		string_sprintf(buffer, "%03X", payload.size());
		buffer.append(";");
		buffer.append(payload);
		buffer.append("\r");

		return buffer;
	}

	friend DTEDecoder;
};


class DTEDecoder {
private:
	static const DTECommandMap *lookup_command(const std::string &command_str, bool is_req) {
		unsigned int start = is_req ? 0 : (unsigned int)DTECommand::__NUM_REQ;
		unsigned int end = is_req ? (unsigned int)DTECommand::__NUM_REQ : command_map_size;
		DEBUG_TRACE("lookup_command: '%s' is_req=%d start=%u end=%u command_map_size=%u", command_str.c_str(), is_req,
		            start, end, (unsigned int)command_map_size);
		for (unsigned int i = start; i < end; i++) {
			if (command_map[i].name == command_str) {
				DEBUG_TRACE("lookup_command: found '%s' at index %u", command_str.c_str(), i);
				return &command_map[i];
			}
		}
		DEBUG_ERROR("DTE_PROTOCOL_UNKNOWN_COMMAND: '%s' not found in range [%u|%u)", command_str.c_str(), start, end);
		throw DTE_PROTOCOL_UNKNOWN_COMMAND;
	}

	static ParamID lookup_key(const std::string &key) {
		auto end = param_map_size;
		for (unsigned int i = 0; i < end; i++) {
			if (param_map[i].key == key) {
				return static_cast<ParamID>(i);
			}
		}
		DEBUG_ERROR("DTE_PROTOCOL_PARAM_KEY_UNRECOGNISED | \"%s\"", key.c_str());
		throw DTE_PROTOCOL_PARAM_KEY_UNRECOGNISED;
	}

	// Returns true if key found, false otherwise (logs warning but doesn't throw)
	static bool try_lookup_key(const std::string &key, ParamID &out_param) {
		auto end = param_map_size;
		for (unsigned int i = 0; i < end; i++) {
			if (param_map[i].key == key) {
				out_param = static_cast<ParamID>(i);
				return true;
			}
		}
		DEBUG_WARN("Unknown parameter key \"%s\" - skipping", key.c_str());
		return false;
	}

	static double decode_frequency(const std::string &s) {
		unsigned int offset_frequency;
		decode(s, offset_frequency);
		double x = ((double)offset_frequency + ARGOS_FREQUENCY_OFFSET) / ARGOS_FREQUENCY_MULT;
		return x;
	}

	static BaseArgosPower decode_power(const std::string &s) {
		if (s == "1") {
			return BaseArgosPower::POWER_3_MW;
		} else if (s == "2") {
			return BaseArgosPower::POWER_40_MW;
		} else if (s == "3") {
			return BaseArgosPower::POWER_200_MW;
		} else if (s == "4") {
			return BaseArgosPower::POWER_500_MW;
		} else if (s == "5") {
			return BaseArgosPower::POWER_5_MW;
		} else if (s == "6") {
			return BaseArgosPower::POWER_50_MW;
		} else if (s == "7") {
			return BaseArgosPower::POWER_350_MW;
		} else if (s == "8") {
			return BaseArgosPower::POWER_750_MW;
		} else if (s == "9") {
			return BaseArgosPower::POWER_1000_MW;
		} else if (s == "10") {
			return BaseArgosPower::POWER_1500_MW;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static unsigned int decode_acquisition_period(const std::string &s) {
		if (s == "0")
			return 0;
		else if (s == "1")
			return 10 * 60;  // 10 min
		else if (s == "2")
			return 15 * 60;  // 15 min
		else if (s == "3")
			return 30 * 60;  // 30 min
		else if (s == "4")
			return 60 * 60;  // 1 hour
		else if (s == "5")
			return 120 * 60;  // 2 hours
		else if (s == "6")
			return 180 * 60;  // 3 hours
		else if (s == "7")
			return 240 * 60;  // 4 hours
		else if (s == "8")
			return 360 * 60;  // 6 hours
		else if (s == "9")
			return 720 * 60;  // 12 hours
		else if (s == "10")
			return 1440 * 60;  // 24 hours
		else if (s == "11")
			return 1 * 60;  // 1 min
		else if (s == "12")
			return 2 * 60;  // 2 min
		else if (s == "13")
			return 5 * 60;  // 5 min
		else if (s == "14")
			return 20 * 60;  // 20 min
		else if (s == "15")
			return 45 * 60;  // 45 min
		else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseGNSSFixMode decode_gnss_fix_mode(const std::string &s) {
		if (s == "1") {
			return BaseGNSSFixMode::FIX_2D;
		} else if (s == "2") {
			return BaseGNSSFixMode::FIX_3D;
		} else if (s == "3") {
			return BaseGNSSFixMode::AUTO;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseLEDMode decode_led_mode(const std::string &s) {
		if (s == "0") {
			return BaseLEDMode::OFF;
		} else if (s == "1") {
			return BaseLEDMode::HRS_24;
		} else if (s == "3") {
			return BaseLEDMode::ALWAYS;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseZoneType decode_zone_type(const std::string &s) {
		if (s == "1") {
			return BaseZoneType::CIRCLE;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseArgosModulation decode_argos_modulation(const std::string &s) {
		if (s == "0") {
			return BaseArgosModulation::LDK;
		} else if (s == "1") {
			return BaseArgosModulation::A2;
		} else if (s == "2") {
			return BaseArgosModulation::A4;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseDebugMode decode_debug_mode(const std::string &s) {
		if (s == "0") {
			return BaseDebugMode::UART;  // 0 = UART debug output
		} else if (s == "1") {
			return BaseDebugMode::USB_CDC;  // 1 = USB CDC debug output
		} else if (s == "2") {
			return BaseDebugMode::BLE_NUS;  // 2 = Bluetooth UART Service
		} else if (s == "3") {
			return BaseDebugMode::NONE;  // 3 = No debug output
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BasePressureSensorLoggingMode decode_pressure_sensor_logging_mode(const std::string &s) {
		if (s == "0") {
			return BasePressureSensorLoggingMode::ALWAYS;
		} else if (s == "1") {
			return BasePressureSensorLoggingMode::UW_THRESHOLD;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BasePressureSensorFullScale decode_pressure_sensor_full_scale(const std::string &s) {
		if (s == "0") {
			return BasePressureSensorFullScale::FS_1260;
		} else if (s == "1") {
			return BasePressureSensorFullScale::FS_4060;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseSensorEnableTxMode decode_sensor_enable_tx_mode(const std::string &s) {
		if (s == "0") {
			return BaseSensorEnableTxMode::OFF;
		} else if (s == "1") {
			return BaseSensorEnableTxMode::ONESHOT;
		} else if (s == "2") {
			return BaseSensorEnableTxMode::MEAN;
		} else if (s == "3") {
			return BaseSensorEnableTxMode::MEDIAN;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseGNSSDynModel decode_gnss_dyn_model(const std::string &s) {
		if (s == "0") {
			return BaseGNSSDynModel::PORTABLE;
		} else if (s == "2") {
			return BaseGNSSDynModel::STATIONARY;
		} else if (s == "3") {
			return BaseGNSSDynModel::PEDESTRIAN;
		} else if (s == "4") {
			return BaseGNSSDynModel::AUTOMOTIVE;
		} else if (s == "5") {
			return BaseGNSSDynModel::SEA;
		} else if (s == "6") {
			return BaseGNSSDynModel::AIRBORNE_1G;
		} else if (s == "7") {
			return BaseGNSSDynModel::AIRBORNE_2G;
		} else if (s == "8") {
			return BaseGNSSDynModel::AIRBORNE_4G;
		} else if (s == "9") {
			return BaseGNSSDynModel::WRIST_WORN_WATCH;
		} else if (s == "10") {
			return BaseGNSSDynModel::BIKE;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static std::time_t decode_datestring(const std::string &s) {
		struct tm tm = {};
		char *p;

		// Try default datestring format
		p = strptime(s.c_str(), "%d/%m/%Y %H:%M:%S", &tm);

		// Try alternative datestring format if this fails
		if (p == nullptr || *p != '\0') p = strptime(s.c_str(), "%a %b %d %H:%M:%S %Y", &tm);

		// Use timegm for UTC interpretation (not affected by local timezone)
		time_t t = timegm(&tm);
		if (t == -1 || p == nullptr || *p != '\0') {
			{
				DEBUG_ERROR("DTE_PROTOCOL_BAD_FORMAT in %s()", __FUNCTION__);
				throw DTE_PROTOCOL_BAD_FORMAT;
			}
		}

		return t;
	}

	static BaseArgosMode decode_mode(const std::string &s) {
		if (s == "0") {
			return BaseArgosMode::OFF;
		} else if (s == "1") {
			return BaseArgosMode::PASS_PREDICTION;
		} else if (s == "2") {
			return BaseArgosMode::LEGACY;
		} else if (s == "3") {
			return BaseArgosMode::DUTY_CYCLE;
		} else if (s == "4") {
			return BaseArgosMode::DOPPLER;
		} else if (s == "5") {
			return BaseArgosMode::SURFACING_BURST;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseUnderwaterDetectSource decode_underwater_detect_source(const std::string &s) {
		if (s == "0") {
			return BaseUnderwaterDetectSource::SWS;
		} else {
			// Only SWS is supported; other sources have been removed
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	static BaseDepthPile decode_depth_pile(const std::string &s) {
		if (s == "1") {
			return BaseDepthPile::DEPTH_PILE_1;
		} else if (s == "2") {
			return BaseDepthPile::DEPTH_PILE_2;
		} else if (s == "3") {
			return BaseDepthPile::DEPTH_PILE_3;
		} else if (s == "4") {
			return BaseDepthPile::DEPTH_PILE_4;
		} else if (s == "8") {
			return BaseDepthPile::DEPTH_PILE_8;
		} else if (s == "9") {
			return BaseDepthPile::DEPTH_PILE_12;
		} else if (s == "10") {
			return BaseDepthPile::DEPTH_PILE_16;
		} else if (s == "11") {
			return BaseDepthPile::DEPTH_PILE_20;
		} else if (s == "12") {
			return BaseDepthPile::DEPTH_PILE_24;
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_VALUE_OUT_OF_RANGE in %s(%s)", __FUNCTION__, s.c_str());
			throw DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
		}
	}

	template <typename T> static inline size_t safe_sscanf(const std::string &s, const char *fmt, T &val) {
		// Provides a way to safely sscanf from a std::string without risk of buffer overrun
		char format[32];
		int written;

		snprintf(format, sizeof(format), "%%%zu", s.size());
		strncat(format, fmt + 1, sizeof(format) - strlen(format) - 1);
		strncat(format, "%n", sizeof(format) - strlen(format) - 1);

		int ret = sscanf(s.c_str(), format, &val, &written);
		if (ret != 1) {
			DEBUG_ERROR("DTE_PROTOCOL_BAD_FORMAT in %s()", __FUNCTION__);
			throw DTE_PROTOCOL_BAD_FORMAT;
		}

		return written;
	}

	static size_t decode(const std::string &s, unsigned int &val, bool is_hex = false) {
		if (is_hex)
			return safe_sscanf(s, "%x", val);
		else
			return safe_sscanf(s, "%u", val);
	}

	static size_t decode(const std::string &s, int &val) { return safe_sscanf(s, "%d", val); }

	static size_t decode(const std::string &s, bool &val) {
		unsigned int temp;
		auto ret = decode(s, temp);
		val = temp;
		return ret;
	}

	static size_t decode(const std::string &s, double &val) { return safe_sscanf(s, "%lf", val); }

	static size_t decode(const std::string &s, std::string &val) {
		val = std::string(s.begin(), std::find(s.begin(), s.end(), ','));
		if (!val.empty()) {
			return val.size();
		} else {
			DEBUG_ERROR("DTE_PROTOCOL_BAD_FORMAT in %s()", __FUNCTION__);
			throw DTE_PROTOCOL_BAD_FORMAT;
		}
	}

	static size_t decode(const std::string &s, std::vector<ParamID> &val, std::vector<std::string> &rejected_keys) {
		constexpr std::string_view delim = ",";

		// Iterate over comma seperated values
		size_t prev = 0;
		size_t pos = 0;
		do {
			pos = s.find(delim, prev);

			if (pos == std::string::npos) pos = s.size();

			auto key = s.substr(prev, pos - prev);
			if (!key.empty()) {
				ParamID param;
				if (try_lookup_key(key, param)) {
					val.push_back(param);
				} else {
					// An unknown key was simply thrown away here, with no trace.
					// Downstream, a request in which NONE of the keys is recognised
					// therefore arrived with an empty list — indistinguishable from
					// an empty request, which means "give me everything". The beacon
					// answered with its ENTIRE configuration to a typo, and a long
					// enough key was enough to force a huge response. We record them
					// so that the caller can tell the difference.
					DEBUG_WARN("DTEDecoder: unknown key \"%s\" ignored", key.c_str());
					rejected_keys.push_back(key);
				}
			}

			prev = pos + delim.size();
		} while (pos < s.length() && prev < s.length());

		return s.size();
	}

	static size_t decode(std::string &s, std::vector<ParamValue> &val, std::vector<std::string> &rejected_keys) {
		// Iterate over comma seperated values
		constexpr std::string_view delim = ",";

		size_t prev = 0;
		size_t pos = 0;
		do {
			pos = s.find(delim, prev);
			if (pos == std::string::npos) pos = s.size();

			auto key_str = s.substr(prev, pos - prev);
			if (!key_str.empty()) {
				auto equals_loc = key_str.find("=");

				if (equals_loc != std::string::npos) {
					ParamValue key_value;
					std::string key = key_str.substr(0, equals_loc);
					std::string value = key_str.substr(equals_loc + 1, std::string::npos);

					// Skip unknown parameters (logs warning but continues processing)
					if (!try_lookup_key(key, key_value.param)) {
						rejected_keys.push_back(key);
						prev = pos + delim.size();
						continue;
					}
					const BaseMap &param_ref = param_map[(unsigned int)key_value.param];
					try {
						switch (param_ref.encoding) {
						case BaseEncoding::DECIMAL: {
							int x;
							decode(value, x);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::HEXADECIMAL: {
							unsigned int x;
							decode(value, x, true);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::UINT: {
							unsigned int x;
							decode(value, x);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::BOOLEAN: {
							bool x;
							decode(value, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::FLOAT: {
							double x;
							decode(value, x);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::TEXT: {
							// This branch was the only one in the parameter decoder that
							// did NOT validate, even though a string validator exists
							// (it checks BASE_TEXT_MAX_LENGTH and the list of allowed
							// values). Measured on the bench on 2026-08-26: 128 characters
							// go through and are persisted, 129 kill the beacon — no
							// answer at all until reset. A single DTE frame was therefore
							// enough to silence it.
							DTEEncoder::validate(param_ref, value);
							key_value.value = value;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::BASE64: {
							key_value.value = websocketpp::base64_decode(value);
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::DATESTRING: {
							std::time_t x = decode_datestring(value);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::UWDETECTSOURCE: {
							BaseUnderwaterDetectSource x = decode_underwater_detect_source(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::ARGOSFREQ: {
							double x = decode_frequency(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::ARGOSPOWER: {
							BaseArgosPower x = decode_power(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::AQPERIOD: {
							unsigned int x = decode_acquisition_period(value);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::DEPTHPILE: {
							BaseDepthPile x = decode_depth_pile(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::ARGOSMODE: {
							BaseArgosMode x = decode_mode(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::GNSSFIXMODE: {
							BaseGNSSFixMode x = decode_gnss_fix_mode(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::GNSSDYNMODEL: {
							BaseGNSSDynModel x = decode_gnss_dyn_model(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::LEDMODE: {
							BaseLEDMode x = decode_led_mode(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::ZONETYPE: {
							BaseZoneType x = decode_zone_type(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::MODULATION: {
							BaseArgosModulation x = decode_argos_modulation(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::DEBUGMODE: {
							BaseDebugMode x = decode_debug_mode(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::PRESSURESENSORLOGGINGMODE: {
							BasePressureSensorLoggingMode x = decode_pressure_sensor_logging_mode(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::PRESSURESENSORFULLSCALE: {
							BasePressureSensorFullScale x = decode_pressure_sensor_full_scale(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::SENSORENABLETXMODE: {
							BaseSensorEnableTxMode x = decode_sensor_enable_tx_mode(value);
							DTEEncoder::validate(param_ref, x);
							key_value.value = x;
							val.push_back(key_value);
							break;
						}
						case BaseEncoding::KEY_LIST:
						case BaseEncoding::KEY_VALUE_LIST:
						default: break;
						}
					} catch (ErrorCode) {
						DEBUG_WARN("Parameter \"%s\" rejected (invalid value \"%s\") - skipping", key.c_str(),
						           value.c_str());
						rejected_keys.push_back(key);
					}
				}
			}

			prev = pos + delim.size();
		} while (pos < s.length() && prev < s.length());

		return s.size();
	}

public:
	// Backward-compatible overload (ignores rejected keys)
	static bool decode(const std::string &str, DTECommand &command, unsigned int &error_code,
	                   std::vector<BaseType> &arg_list, std::vector<ParamID> &keys,
	                   std::vector<ParamValue> &key_values) {
		std::vector<std::string> rejected_keys;
		return decode(str, command, error_code, arg_list, keys, key_values, rejected_keys);
	}

	static bool decode(const std::string &str, DTECommand &command, unsigned int &error_code,
	                   std::vector<BaseType> &arg_list, std::vector<ParamID> &keys, std::vector<ParamValue> &key_values,
	                   std::vector<std::string> &rejected_keys) {
		constexpr std::string_view cmd_resp_ok("$O;");
		constexpr std::string_view cmd_resp_nok("$N;");
		constexpr std::string_view cmd_req("$");
		constexpr std::string_view length_deliminator("#");
		constexpr std::string_view command_deliminator(";");
		constexpr std::string_view payload_end_deliminator("\r");
		constexpr size_t size_of_command_field =
		    8;  // Max command name length. Increased from 6 to support GNSSBCKP (8 chars) and ARGOSTX (7 chars), which were silently rejected by the sanity check before reaching lookup_command().
		constexpr size_t size_of_length_field = 3;

		bool is_req = false;
		bool cmd_nok = false;
		size_t str_pos = 0;

		error_code = 0;

		// Check the message header to determine what message type this is //
		if (str.compare(str_pos, cmd_resp_ok.size(), cmd_resp_ok) == 0) {
			str_pos += cmd_resp_ok.size();
			is_req = false;
		} else if (str.compare(str_pos, cmd_resp_nok.size(), cmd_resp_nok) == 0) {
			str_pos += cmd_resp_nok.size();
			is_req = false;
			cmd_nok = true;
		} else if (str.compare(str_pos, cmd_req.size(), cmd_req) == 0) {
			str_pos += cmd_req.size();
			is_req = true;
		} else {
			return false;
		}

		// Extract the command by finding the '#' delimiter (variable length command names)
		size_t cmd_end_pos = str.find(length_deliminator, str_pos);
		if (cmd_end_pos == std::string::npos || cmd_end_pos == str_pos) return false;
		size_t cmd_len = cmd_end_pos - str_pos;
		if (cmd_len > size_of_command_field)  // Sanity check - command name too long
			return false;
		const DTECommandMap *cmd_ref = lookup_command(std::string(str.substr(str_pos, cmd_len)), is_req);
		str_pos = cmd_end_pos + length_deliminator.size();
		command = cmd_ref->command;

		// Extract the length field //

		// Check that our sscanf function won't buffer overrun
		if (str_pos + size_of_length_field >= str.size()) return false;

		// The length field is EXACTLY size_of_length_field hexadecimal
		// characters. sscanf alone does not guarantee that, and the two ways it
		// fails were measured on the bench on 2026-08-26:
		//   "ZZZ" -> sscanf converts nothing, returns 0, and leaves `length`
		//            UNINITIALISED. The comparison further down therefore ran
		//            on an indeterminate value: 15 identical sends produced two
		//            different responses.
		//   "00;" -> sscanf succeeds while consuming only TWO characters. Blindly
		//            advancing by three then skipped the delimiter, the next
		//            check failed, and the function returned false — that is,
		//            NO RESPONSE AT ALL. The host stayed blocked, waiting.
		// We therefore validate the three characters before converting, and raise
		// a format error: the command name is already known at this point, so the
		// response can be named and the host is not left waiting for nothing.
		size_t length = 0;
		for (unsigned int k = 0; k < size_of_length_field; k++) {
			if (!isxdigit(static_cast<unsigned char>(str[str_pos + k]))) {
				DEBUG_ERROR("DTE_PROTOCOL_BAD_FORMAT: length field is not hexadecimal");
				throw DTE_PROTOCOL_BAD_FORMAT;
			}
		}
		if (sscanf(&str[str_pos], "%3zX", &length) != 1) {
			DEBUG_ERROR("DTE_PROTOCOL_BAD_FORMAT: length field unreadable");
			throw DTE_PROTOCOL_BAD_FORMAT;
		}
		str_pos += size_of_length_field;

		// Check the command deliminator //
		if (str.compare(str_pos, command_deliminator.size(), command_deliminator) != 0) return false;
		str_pos += command_deliminator.size();

		// Check the final character is a character return //
		if (str.compare(str.size() - payload_end_deliminator.size(), payload_end_deliminator.size(),
		                payload_end_deliminator)
		    != 0)
			return false;

		size_t payload_len = str.size() - str_pos - payload_end_deliminator.size();

		// Check the supplied length matches the actual length //
		if (length != payload_len) {
			DEBUG_ERROR("DTE_PROTOCOL_PAYLOAD_LENGTH_MISMATCH | expected %ld but got %ld", length, payload_len);
			throw DTE_PROTOCOL_PAYLOAD_LENGTH_MISMATCH;
		}

		// Check the received message is not too large //
		if (payload_len > BASE_MAX_PAYLOAD_LENGTH) {
			DEBUG_ERROR("DTE_PROTOCOL_MESSAGE_TOO_LARGE");
			throw DTE_PROTOCOL_MESSAGE_TOO_LARGE;
		}

		// If this is a NOK message, retrieve the error code //
		if (cmd_nok) {
			sscanf(&str[str_pos], "%1u", &error_code);
			str_pos += 1;
		}

		if (error_code == 0) {
			std::string payload = str.substr(str_pos, str.size() - str_pos - payload_end_deliminator.size());

			size_t payload_pos = 0;

			// KEY_LIST is permitted to be zero length (= return all params)
			if (cmd_ref->prototype.size() && !payload_len && cmd_ref->prototype[0].encoding != BaseEncoding::KEY_LIST) {
				DEBUG_ERROR("DTE_PROTOCOL_MISSING_ARG");
				throw DTE_PROTOCOL_MISSING_ARG;
			}

			if (payload_len && !cmd_ref->prototype.size()) {
				DEBUG_ERROR("DTE_PROTOCOL_UNEXPECTED_ARG");
				throw DTE_PROTOCOL_UNEXPECTED_ARG;
			}

			// Skip decode loop when payload is empty and KEY_LIST (= all params)
			if (!payload_len && cmd_ref->prototype.size() && cmd_ref->prototype[0].encoding == BaseEncoding::KEY_LIST) {
				// keys/key_values stay empty → handler returns all params
			}

			// Iterate over expected parameters based on the command map entries
			else
				for (unsigned int arg_index = 0; arg_index < cmd_ref->prototype.size(); arg_index++) {
					if (arg_index > 0) {
						// Skip over parameter separator and check it is a "," character
						if (payload_pos >= payload.size()) {
							// Allow optional trailing args when min_args is set
							if (cmd_ref->min_args > 0 && arg_index >= cmd_ref->min_args) {
								break;
							}
							DEBUG_ERROR("DTE_PROTOCOL_MISSING_ARG");
							throw DTE_PROTOCOL_MISSING_ARG;
						}

						char x = payload[payload_pos];
						payload_pos++;
						if (x != ',') {
							DEBUG_ERROR("DTE_PROTOCOL_BAD_FORMAT in %s()", __FUNCTION__);
							throw DTE_PROTOCOL_BAD_FORMAT;
						}
					}

					auto remaining_str = payload.substr(payload_pos, std::string::npos);

					switch (cmd_ref->prototype[arg_index].encoding) {
					case BaseEncoding::KEY_VALUE_LIST:
						DEBUG_TRACE("BaseEncoding::KEY_VALUE_LIST");
						payload_pos += decode(remaining_str, key_values, rejected_keys);
						break;
					case BaseEncoding::KEY_LIST:
						DEBUG_TRACE("BaseEncoding::KEY_LIST");
						payload_pos += decode(remaining_str, keys, rejected_keys);
						break;
					case BaseEncoding::DECIMAL: {
						DEBUG_TRACE("BaseEncoding::DECIMAL");
						int val;
						payload_pos += decode(remaining_str, val);
						DTEEncoder::validate(cmd_ref->prototype[arg_index], val);
						arg_list.push_back(val);
						break;
					}
					case BaseEncoding::HEXADECIMAL: {
						DEBUG_TRACE("BaseEncoding::HEXADECIMAL");
						unsigned int val;
						payload_pos += decode(remaining_str, val, true);
						DTEEncoder::validate(cmd_ref->prototype[arg_index], val);
						arg_list.push_back(val);
						break;
					}
					case BaseEncoding::UINT: {
						DEBUG_TRACE("BaseEncoding::UINT");
						unsigned int val;
						payload_pos += decode(remaining_str, val);
						DTEEncoder::validate(cmd_ref->prototype[arg_index], val);
						arg_list.push_back(val);
						break;
					}
					case BaseEncoding::BOOLEAN: {
						DEBUG_TRACE("BaseEncoding::BOOLEAN");
						bool val;
						payload_pos += decode(remaining_str, val);
						arg_list.push_back(val);
						break;
					} break;
					case BaseEncoding::FLOAT: {
						DEBUG_TRACE("BaseEncoding::FLOAT");
						double val;
						payload_pos += decode(remaining_str, val);
						DTEEncoder::validate(cmd_ref->prototype[arg_index], val);
						arg_list.push_back(val);
						break;
					}
					case BaseEncoding::TEXT: {
						DEBUG_TRACE("BaseEncoding::TEXT");
						std::string val;
						payload_pos += decode(remaining_str, val);
						arg_list.push_back(val);
						break;
					}
					case BaseEncoding::BASE64: {
						DEBUG_TRACE("BaseEncoding::BASE64");
						std::string val;
						payload_pos += decode(remaining_str, val);
						arg_list.push_back(websocketpp::base64_decode(val));
						break;
					}
					case BaseEncoding::DATESTRING:
					case BaseEncoding::DEPTHPILE:
					case BaseEncoding::ARGOSMODE:
					case BaseEncoding::ARGOSPOWER:
					case BaseEncoding::AQPERIOD:
					case BaseEncoding::ARGOSFREQ:
					case BaseEncoding::UWDETECTSOURCE:
					case BaseEncoding::GNSSFIXMODE:
					case BaseEncoding::GNSSDYNMODEL:
					case BaseEncoding::LEDMODE:
					case BaseEncoding::ZONETYPE:
					case BaseEncoding::MODULATION:
					case BaseEncoding::DEBUGMODE:
					case BaseEncoding::PRESSURESENSORLOGGINGMODE:
					case BaseEncoding::PRESSURESENSORFULLSCALE:
					case BaseEncoding::SENSORENABLETXMODE:
					default: DEBUG_ERROR("BaseEncoding::Not supported"); break;
					}
				}
		}

		return true;
	}
};
