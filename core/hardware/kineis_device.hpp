/**
 * @file kineis_device.hpp
 * @brief Abstract satellite/LoRa device interface — TX, RX, credentials, modulation switching.
 */

#pragma once

#include <cstdint>
#include <list>
#include <string>

using KineisPacket = std::string;

struct KineisEventPowerOn {};
struct KineisEventPowerOff {};
struct KineisEventTxStarted {};
struct KineisEventTxComplete {};
struct KineisEventRxStarted {};
struct KineisEventRxStopped {
	unsigned int rx_time;
};
struct KineisEventRxPacket {
	KineisPacket packet;
	unsigned int size_bits;
};
struct KineisEventDeviceError {};

enum KineisModulation { LDK, LDA2, VLDA4 };

class KineisEventListener {
public:
	virtual void react(KineisEventPowerOn const &) {}
	virtual void react(KineisEventPowerOff const &) {}
	virtual void react(KineisEventTxStarted const &) {}
	virtual void react(KineisEventTxComplete const &) {}
	virtual void react(KineisEventRxStarted const &) {}
	virtual void react(KineisEventRxStopped const &) {}
	virtual void react(KineisEventRxPacket const &) {}
	virtual void react(KineisEventDeviceError const &) {}
};


/// @brief Abstract satellite/LoRa device — implemented by SmdSat, KIM2Device, LoRaDevice.
class KineisDevice {
private:
	std::list<KineisEventListener *> m_listeners;

protected:
	unsigned int m_idle_timeout_ms = 1000;

public:
	virtual ~KineisDevice() {}
	void subscribe(KineisEventListener &m) { m_listeners.push_back(&m); }
	void unsubscribe(KineisEventListener &m) { m_listeners.remove(&m); }
	template <typename E> void notify(E const &e) {
		for (auto m : m_listeners) {
			m->react(e);
		}
	}
	void set_idle_timeout(unsigned int ms) { m_idle_timeout_ms = ms; }
	virtual void send(const KineisModulation mode, const KineisPacket &packet, const unsigned int size_bits) = 0;
	virtual void stop_send() = 0;
	virtual void power_off_immediate() {}  // Hard shutdown (default: no-op for backends without power control)
	/// @brief Pre-boot the radio module without sending anything. Lets the
	/// next send() dispatch immediately instead of paying the boot+configure
	/// cost. Default no-op for backends where wake is already fast enough.
	virtual void warm_up_for_tx() {}
	/// @brief Whether the backend could actually put a frame on the air now.
	/// Consulted BEFORE the depth pile is retrieved, and that ordering is the
	/// whole point: retrieve() spends a burst credit, and an entry back at zero
	/// is never eligible again, so a fix consumed by a transmission that could
	/// never happen is lost for good -- with NTRY=1, on the first try. Backends
	/// that are always able to transmit keep the default.
	virtual bool can_transmit() const { return true; }
	virtual void start_receive(const KineisModulation mode) = 0;
	virtual bool stop_receive() = 0;

	/// @brief True while the backend is receiving (or about to), i.e. unable to
	///        transmit. Lets the TX service defer instead of consuming a depth
	///        pile redundancy slot on a transmission that cannot leave.
	///        Default false for backends with no receive path.
	virtual bool is_receiving() const { return false; }
	virtual void set_frequency(double freq_mhz) = 0;
	virtual void set_tcxo_warmup_time(unsigned int ms) = 0;
	virtual void set_tx_power(unsigned int power) { (void)power; }
	virtual void set_lpm_mode(uint8_t lpm_bitmap) { (void)lpm_bitmap; }
	virtual void set_credentials(unsigned int dec_id, unsigned int address, const std::string &seckey,
	                             const std::string &radioconf) {
		(void)dec_id;
		(void)address;
		(void)seckey;
		(void)radioconf;
	}
	virtual void read_credentials(unsigned int *dec_id, unsigned int *address, std::string *seckey,
	                              std::string *radioconf) {
		(void)dec_id;
		(void)address;
		(void)seckey;
		(void)radioconf;
	}
	/// @brief Program the configured RCONF, read it back, and re-seed the
	/// modulation cache so the next TX is sized for the radio's true modulation.
	/// Called after a master-RCONF edit (PARMW) or force-SATVF. Default no-op:
	/// SMD re-pushes via its own credentials-dirty flow; only KIM2 needs this.
	virtual bool resync_rconf_cache() { return false; }
	// Switch radio modulation at runtime: write RCONF, save to flash, reload KMAC.
	// rconf_hex: 32-char hex string for the target modulation's radio configuration.
	// Returns true on success.
	virtual bool switch_modulation(KineisModulation mode, const std::string &rconf_hex) {
		(void)mode;
		(void)rconf_hex;
		return true;
	}
	virtual KineisModulation get_current_modulation() const { return KineisModulation::LDA2; }
	/// @brief Milliseconds remaining on the device's internal error cooldown,
	/// or 0 if the device is not in cooldown. Used by ArgosTxService to
	/// distinguish "cooldown reject" from a real device error so the
	/// session-suspension budget isn't burned on the autofallback recovery
	/// window. Default: backend has no cooldown concept.
	virtual unsigned int cooldown_remaining_ms() const { return 0; }
};