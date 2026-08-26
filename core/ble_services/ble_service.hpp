/**
 * @file ble_service.hpp
 * @brief Abstract BLE service interface — DTE data, OTA events, connection lifecycle.
 */

#pragma once

#include <functional>
#include <string>

enum class BLEServiceEventType {
	CONNECTED,
	DISCONNECTED,
	DTE_DATA_RECEIVED,
	OTA_START,
	OTA_FILE_DATA,
	OTA_ABORT,
	OTA_END
};

struct BLEServiceEvent {
	BLEServiceEventType event_type;
	union {
		struct {
			unsigned int file_id;
			unsigned int file_size;
			unsigned int crc32;
		};
		struct {
			void *data;
			unsigned int length;
		};
	};
};

class BLEService {
public:
	virtual ~BLEService() {}
	virtual void init() {}
	virtual void start(std::function<int(BLEServiceEvent& event)> on_event) = 0;
	virtual void stop() = 0;
	virtual bool write(std::string str) = 0;
	virtual bool write_best_effort(std::string str) { return write(str); }
	virtual std::string read_line() = 0;
	virtual void set_device_name(const std::string&) = 0;
#ifdef BENCH_TEST
	/// @brief Banc uniquement: l'advertising est-il encore voulu/actif ?
	/// Permet de VERIFIER sur la carte qu'apres une sortie du mode configuration
	/// la radio reste bien eteinte, au lieu de le deduire du code.
	virtual bool bench_is_advertising() { return false; }
	/// @brief Banc: mode advertising vu par le module SDK (0 = IDLE).
	/// ATTENTION: sd_ble_gap_adv_stop() NE remet PAS ce champ a IDLE — il reste
	/// fige sur FAST apres un arret. Il ne prouve donc rien a lui seul.
	virtual int bench_adv_mode() { return -1; }
	/// @brief Banc: sonde AUTORITATIVE aupres du SoftDevice.
	/// Renvoie le code retour de sd_ble_gap_adv_stop(): 0 = on advertissait
	/// VRAIMENT (et on vient de l'arreter), NRF_ERROR_INVALID_STATE (8) = on
	/// n'advertissait pas. Effet de bord assume: coupe l'advertising.
	virtual unsigned int bench_probe_advertising() { return 0xFFFFFFFF; }
	/// @brief Banc uniquement: injecte un evenement de deconnexion BLE synthetique
	/// pour rejouer, sans telephone, la sequence qui laissait l'advertising rallume.
	virtual void bench_inject_disconnect() {}
#endif
};
