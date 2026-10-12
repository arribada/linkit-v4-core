#pragma once

#include <cstdint>
#include <cstring>
#include <string>

/// @brief Host-owned Argos MAC message counter (MC, 9 bits, 0-511).
///
/// The module numbers messages itself, but it cannot keep the count: the SMD
/// driver cuts the module's rail after EVERY transmission, and the module keeps
/// its counter in RTC backup registers (argos-smd-at-kineis-firmware,
/// mcu_nvm.c / .msgCntSectionData), so each power-on starts it again from
/// whatever the backup domain still holds. The host therefore decides the MC
/// and writes it before every transmission (AT+MC / SPI 0x2D-0x2E).
///
/// Operator rule (2026-10-12):
///  - every Doppler frame of ONE surfacing carries the same MC;
///  - a frame carrying new information (GNSS position, sensors...) takes a new MC;
///  - an identical repeat of such a frame (NTRY, pile rotation, heartbeat of the
///    same cached position) keeps the MC it was first sent with;
///  - a new surfacing starts afresh: new Doppler MC, and repeats of last
///    surfacing's frames are new messages.
/// Outside a surfacing burst there is no surfacing to group by, so Doppler
/// frames fall back to the repeat rule (identical bytes, same MC).
///
/// The running counter survives a warm reset (.noinit + CRC). After a cold
/// boot it is reseeded from the clock when it is set, so a reset does not
/// restart every tag at 0 and reuse the counters it just sent.
class ArgosMessageCounter {
public:
	static constexpr uint16_t MC_MODULO = 512;
	static constexpr unsigned int REPEAT_CACHE = 8;

	enum class Kind { NONE, DOPPLER, DATA };

	/// @brief Classify a frame from the type tag ArgosTxService sets before send().
	static Kind kind_of(const char *val_tx_type) {
		if (!val_tx_type) return Kind::NONE;
		if (std::strcmp(val_tx_type, "cert") == 0 || std::strcmp(val_tx_type, "none") == 0) return Kind::NONE;
		if (std::strncmp(val_tx_type, "doppler", 7) == 0) return Kind::DOPPLER;  // doppler, doppler-prewarm
		return Kind::DATA;
	}

	/// @brief Restore the running counter after a reset. `now` = 0 when the clock is unknown.
	void restore(uint32_t now) {
		if (s_state.magic == MAGIC && s_state.crc == crc(s_state) && s_state.next < MC_MODULO) return;
		s_state.magic = MAGIC;
		s_state.next = static_cast<uint16_t>(now ? (now / 60U) % MC_MODULO : 0U);
		s_state.crc = crc(s_state);
	}

	/// @brief A new surfacing: new Doppler MC, and last surfacing's frames are no longer repeats.
	void on_surfaced() {
		m_doppler_valid = false;
		m_cache_n = 0;
		m_cache_head = 0;
	}

	/// @brief The MC this frame must carry.
	uint16_t assign(Kind kind, const std::string &payload, bool surfacing_burst) {
		if (kind == Kind::DOPPLER && surfacing_burst) {
			if (!m_doppler_valid) {
				m_doppler_mc = take_next();
				m_doppler_valid = true;
			}
			return m_doppler_mc;
		}
		for (unsigned int i = 0; i < m_cache_n; i++) {
			if (m_cache[i].payload == payload) return m_cache[i].mc;
		}
		const uint16_t mc = take_next();
		m_cache[m_cache_head].payload = payload;
		m_cache[m_cache_head].mc = mc;
		m_cache_head = (m_cache_head + 1) % REPEAT_CACHE;
		if (m_cache_n < REPEAT_CACHE) m_cache_n++;
		return mc;
	}

	uint16_t peek_next() const { return s_state.next; }

private:
	static constexpr uint32_t MAGIC = 0x4D433031;  // "MC01"

	struct State {
		uint32_t magic;
		uint16_t next;
		uint16_t crc;
	};

	static uint16_t crc(const State &s) {
		// Not a real CRC: enough to reject the random contents of .noinit after a cold boot.
		return static_cast<uint16_t>((s.magic ^ (s.magic >> 16) ^ s.next ^ 0xA55AU) & 0xFFFFU);
	}

	uint16_t take_next() {
		const uint16_t mc = s_state.next;
		s_state.next = static_cast<uint16_t>((s_state.next + 1U) % MC_MODULO);
		s_state.crc = crc(s_state);
		return mc;
	}

	struct Entry {
		std::string payload;
		uint16_t mc = 0;
	};

	Entry m_cache[REPEAT_CACHE];
	unsigned int m_cache_n = 0;
	unsigned int m_cache_head = 0;
	bool m_doppler_valid = false;
	uint16_t m_doppler_mc = 0;

#ifndef CPPUTEST
	inline static State s_state __attribute__((section(".noinit")));
#else
public:
	inline static State s_state = { 0, 0, 0 };  // tests reset it directly
#endif
};
