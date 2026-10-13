/**
 * @file ubx.hpp
 * @brief ubx driver.
 */

#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include <new>  // placement new for safe MSG_VALSET construction

namespace UBX {
static constexpr uint8_t SYNC_CHAR1 = 0xB5;
static constexpr uint8_t SYNC_CHAR2 = 0x62;
static constexpr size_t MAX_PACKET_LEN = 256;

enum class MessageClass : uint8_t {
	MSG_CLASS_BAD = 0x00,
	MSG_CLASS_NAV = 0x01,
	MSG_CLASS_RXM = 0x02,
	MSG_CLASS_INF = 0x04,
	MSG_CLASS_ACK = 0x05,
	MSG_CLASS_CFG = 0x06,
	MSG_CLASS_MON = 0x0A,
	MSG_CLASS_AID = 0x0B,
	MSG_CLASS_TIM = 0x0D,
	MSG_CLASS_MGA = 0x13,
	MSG_CLASS_LOG = 0x21,
	MSG_CLASS_SEC = 0x27
};

struct __attribute__((__packed__)) Empty {};

struct __attribute__((__packed__)) Header {
	uint8_t syncChars[2];
	MessageClass msgClass;
	uint8_t msgId;
	uint16_t msgLength;  // Excludes header and CRC bytes
};

template <typename T> static unsigned int msg_size() {
	return sizeof(T) + sizeof(Header) + 2;
}

struct __attribute__((__packed__)) HeaderAndPayloadCRC {
	uint8_t syncChars[2];
	MessageClass msgClass;
	uint8_t msgId;
	uint16_t msgLength;  // Excludes header and CRC bytes
	uint8_t payload[MAX_PACKET_LEN - sizeof(Header)];
};

/****************************** ACK *************************************/

namespace ACK {
enum Id : uint8_t { ID_NACK = 0x00, ID_ACK = 0x01 };

struct __attribute__((__packed__)) MSG_ACK {
	MessageClass clsID;
	uint8_t msgID;
};

struct __attribute__((__packed__)) MSG_NACK {
	MessageClass clsID;
	uint8_t msgID;
};

}  // namespace ACK

/****************************** MON *************************************/

namespace MON {
enum Id : uint8_t { ID_VER = 0x04 };

struct __attribute__((__packed__)) MSG_VER {
	char swVersion[30];
	char hwVersion[10];
	// Variable-length extension strings follow (30 bytes each)
};

}  // namespace MON

/****************************** SEC *************************************/

namespace SEC {
enum Id : uint8_t { ID_UNIQID = 0x03 };

struct __attribute__((__packed__)) MSG_UNIQID {
	uint8_t version;
	uint8_t reserved[3];
	uint8_t uniqueId[5];
};

}  // namespace SEC

/****************************** CFG *************************************/

namespace CFG {
struct UBXParameter {
	uint32_t key;
	uint8_t size;
	std::vector<uint8_t> value;

	// Set value according to the required size (automatically adjusts byte order)
	template <typename T> void set_value(T val) {
		value.resize(size);
		for (int i = 0; i < size; ++i) {
			value[i] = static_cast<uint8_t>((val >> (8 * i)) & 0xFF);
		}
	}

	// Get value according to the required size and type (automatically adjusts byte order)
	template <typename T> T get_value() const {
		T val = 0;
		for (int i = 0; i < size && i < sizeof(T); ++i) {
			val |= static_cast<T>(value[i]) << (8 * i);
		}
		return val;
	}
};

enum Id : uint8_t {
	ID_CFG = 0x09,
	ID_RST = 0x04,
	ID_VALDEL = 0x8c,
	ID_VALGET = 0x8b,
	ID_VALSET = 0x8a,

	ID_PRT = 0x00,
	ID_ANT = 0x13,
	ID_DAT = 0x06,
	ID_GNSS = 0x3E,
	ID_INF = 0x02,
	ID_ITFM = 0x39,
	ID_LOGFILTER = 0x47,
	ID_MSG = 0x01,
	ID_NAV5 = 0x24,
	ID_NAVX5 = 0x23,
	ID_NMEA = 0x17,
	ID_NVS = 0x22,
	ID_PM2 = 0x3B,
	ID_RATE = 0x08,
	ID_RINV = 0x34,
	//ID_RST       = 0x04,
	ID_RXM = 0x11,
	ID_SBAS = 0x16,
	ID_TP5 = 0x31,
	ID_USB = 0x1B,
	ID_ODO = 0x1E
};

namespace RST {
enum ResetMode : uint8_t {
	RESETMODE_HARDWARE_RESET_IMMEDIATE = 0x00,
	RESETMODE_SOFTWARE_RESET = 0x01,
	RESETMODE_SOFTWARE_RESET_GNSS_ONLY = 0x02,
	RESETMODE_HARDWARE_RESET_AFTER_SHUTDOWN = 0x04,
	RESETMODE_CONTROLLED_GNSS_STOP = 0x08,
	RESETMODE_CONTROLLED_GNSS_START = 0x09
};

struct __attribute__((__packed__)) MSG_RST {
	uint16_t navBbrMask;
	ResetMode resetMode;
	uint8_t reserved1;
};
}  // namespace RST

namespace VALDEL {
enum LAYERS : uint8_t { BBR = 1 << 0, FLASH = 1 << 1 };
// FIXME: Same flexible array member UB as MSG_VALSET (see comment there).
struct __attribute__((__packed__)) MSG_VALDEL {
	uint8_t version;
	uint8_t layers;
	uint16_t reserved0;
	uint32_t keys[];
	// Constructor for easy initialization
	MSG_VALDEL(uint8_t version_, uint8_t layers_, const std::vector<uint32_t> &key_ids)
	    : version(version_),
	      layers(layers_),
	      reserved0(0) {
		// Allocate memory for keys dynamically after creating MSG_RST instance
		uint32_t *key_ptr = reinterpret_cast<uint32_t *>(this + 1);
		std::copy(key_ids.begin(), key_ids.end(), key_ptr);
	}
};
}  // namespace VALDEL

namespace VALGET {
enum LAYERS : uint8_t {
	RAM = 0,     // RAM layer
	BBR = 1,     // Battery-backed RAM
	FLASH = 2,   // Flash storage
	DEFAULT = 7  // Default configuration
};

struct __attribute__((__packed__)) MSG_VALGET {
	uint8_t version;    // Message version (0x01)
	uint8_t layer;      // Layer to retrieve from (0 - RAM, 1 - BBR, etc.)
	uint16_t position;  // Skipped config items in the result set
	uint32_t keys[];    // Flexible array for dynamic key retrieval

	// Constructor to initialize MSG_VALGET
	MSG_VALGET(uint8_t version_, LAYERS layer_, uint16_t position_, const std::vector<uint32_t> &key_ids)
	    : version(version_),
	      layer(layer_),
	      position(position_) {
		// Copy keys into the dynamic array after MSG_VALGET
		uint32_t *key_ptr = reinterpret_cast<uint32_t *>(this + 1);
		std::copy(key_ids.begin(), key_ids.end(), key_ptr);
	}
};
struct __attribute__((__packed__)) MSG_VALGET_RESP {
	uint8_t version;    // Message version (0x01)
	uint8_t layer;      // Layer to retrieve from (0 - RAM, 1 - BBR, etc.)
	uint16_t position;  // Skipped config items in the result set
	uint8_t cfgData[];  // Flexible array for dynamic key retrieval
};
}

namespace VALSET {
// Layer bitfield definitions for configuration updates
enum LAYERS : uint8_t {
	RAM = 1 << 0,   // Update in RAM layer
	BBR = 1 << 1,   // Update in BBR layer
	FLASH = 1 << 2  // Update in Flash layer
};

// Max cfgData bytes for a single VALSET message. Bounded by the UBX
// packet buffer (MAX_PACKET_LEN=256) minus header/CRC; the real
// configs use <= ~112 B (14 key/value pairs), so this is generous.
static constexpr size_t MSG_VALSET_MAX_CFG = 256;

// MSG_VALSET has a flexible array member (cfgData[]): sizeof == 4 and
// the constructor fills the key/value bytes via `this + 1`. It MUST
// therefore be constructed with placement-new into a buffer that has
// room for the header (4 B) + all key/value bytes — NOT as a bare
// stack local, which is only 4 B and whose constructor overflows the
// stack (UB; "worked by luck" on GCC 10.3 ARM until an ASan run in
// 2026-07 caught it). Every call site now does:
//   alignas(MSG_VALSET) uint8_t buf[sizeof(MSG_VALSET) + MSG_VALSET_MAX_CFG];
//   auto& msg = *new (buf) MSG_VALSET(version, layers, params);
// send_packet_with_expect still copies sizeof(MSG_VALSET)+dynamic_size
// = 4 + cfgDataSize bytes, so only the header + used cfgData go on air.
struct __attribute__((__packed__)) MSG_VALSET {
	uint8_t version;       // Message version (0x00)
	uint8_t layers;        // Bitfield for layer selection (e.g., RAM, BBR, FLASH)
	uint8_t reserved0[2];  // Reserved bytes
	uint32_t cfgData[];    // Flexible array for dynamic key-value pairs

	// Constructor to dynamically add key-value pairs
	MSG_VALSET(uint8_t version_, uint8_t layers_, const std::vector<UBXParameter> &params)
	    : version(version_),
	      layers(layers_) {
		reserved0[0] = 0;
		reserved0[1] = 0;

		// Write each key-value pair into the cfgData array
		uint8_t *data_ptr = reinterpret_cast<uint8_t *>(this + 1);
		for (const auto &param : params) {
			// Write the key (4 bytes)
			*reinterpret_cast<uint32_t *>(data_ptr) = param.key;
			data_ptr += sizeof(uint32_t);

			// Write the value according to its specified size
			for (uint8_t byte : param.value) {
				*data_ptr++ = byte;
			}
		}
	}
	// Method to calculate cfgData size in bytes
	size_t get_cfgData_size(const std::vector<UBXParameter> &params) const {
		size_t size = 0;
		for (const auto &param : params) {
			size += sizeof(param.key) + param.value.size();
		}
		return size;
	}
};
}

namespace UART1 {
// Key IDs for UART1 Configuration
inline UBXParameter BAUDRATE = { 0x40520001, 4, {} };  // U4
inline UBXParameter STOPBITS = { 0x20520002, 1, {} };  // E1
inline UBXParameter DATABITS = { 0x20520003, 1, {} };  // E1
inline UBXParameter PARITY = { 0x20520004, 1, {} };    // E1
inline UBXParameter ENABLED = { 0x10520005, 1, {} };   // L

// UART1 Input Protocol Flags
inline UBXParameter INPROT_UBX = { 0x10730001, 1, {} };   // Flag to enable UBX as input protocol - 1 byte
inline UBXParameter INPROT_NMEA = { 0x10730002, 1, {} };  // Flag to enable NMEA as input protocol - 1 byte

// UART1 Output Protocol Flags
inline UBXParameter OUTPROT_UBX = { 0x10740001, 1, {} };   // Flag to enable UBX as output protocol - 1 byte
inline UBXParameter OUTPROT_NMEA = { 0x10740002, 1, {} };  // Flag to enable NMEA as output protocol - 1 byte

// TX Ready Parameters
inline UBXParameter TXREADY_ENABLED = { 0x10a20001, 1, {} };    // TX ready enable flag - 1 byte
inline UBXParameter TXREADY_POLARITY = { 0x10a20002, 1, {} };   // TX ready polarity - 1 byte
inline UBXParameter TXREADY_PIN = { 0x20a20003, 1, {} };        // TX ready pin number - 1 byte
inline UBXParameter TXREADY_THRESHOLD = { 0x30a20004, 2, {} };  // TX ready threshold - 2 bytes
inline UBXParameter TXREADY_INTERFACE = { 0x20a20005, 1, {} };  // TX ready interface - 1 byte
// Enumerations for possible values of stop bits, data bits, and parity mode
enum StopBits : uint8_t {
	HALF = 0,     // 0.5 stop bits
	ONE = 1,      // 1.0 stop bits
	ONEHALF = 2,  // 1.5 stop bits
	TWO = 3       // 2.0 stop bits
};

enum DataBits : uint8_t {
	EIGHT = 0,  // 8 data bits
	SEVEN = 1   // 7 data bits
};

enum Parity : uint8_t {
	NONE = 0,  // No parity bit
	ODD = 1,   // Odd parity bit
	EVEN = 2   // Even parity bit
};
}

namespace MSG {
inline UBXParameter MSG_CLASS = { 0x20910001, 1, {} };  // Message class (1 byte)
inline UBXParameter MSG_ID = { 0x20910002, 1, {} };     // Message ID (1 byte)
inline UBXParameter RATE = { 0x20910003, 1, {} };       // Message rate (1 byte)
struct __attribute__((__packed__)) MSG_MSG {
	MessageClass msgClass;
	uint8_t msgID;
	uint8_t rate;
};
struct __attribute__((__packed__)) MSG_MSG_NORATE {
	MessageClass msgClass;
	uint8_t msgID;
};
}  // namespace MSG

namespace MSGOUT {
inline UBXParameter NAV_PVT_UART1 = { 0x20910007, 1, {} };     // Output rate of the UBX-NAV-PVT message on UART1
inline UBXParameter NAV_DOP_UART1 = { 0x20910039, 1, {} };     // Output rate of the UBX-NAV-DOP message on UART1
inline UBXParameter NAV_STATUS_UART1 = { 0x2091001b, 1, {} };  // Output rate of the UBX-NAV-STATUS message on UART1
inline UBXParameter NAV_SAT_UART1 = { 0x20910016, 1, {} };     // Output rate of the UBX-NAV-SAT message on UART1

// CloudLocate raw measurement output rates on UART1
// Key IDs from u-blox M10 SPG 5.10 interface description (UBX-21035062)
inline UBXParameter RXM_MEASC12_UART1 = { 0x2091063F, 1, {} };  // CFG-MSGOUT-UBX_RXM_MEASC12_UART1
inline UBXParameter RXM_MEAS20_UART1 = { 0x20910644, 1, {} };   // CFG-MSGOUT-UBX_RXM_MEAS20_UART1
inline UBXParameter RXM_MEAS50_UART1 = { 0x20910649, 1, {} };   // CFG-MSGOUT-UBX_RXM_MEAS50_UART1
}


namespace SIGNAL {
inline UBXParameter GPS_ENA = { 0x1031001F, 1, {} };        // L (1 byte) GPS enable
inline UBXParameter GPS_L1CA_ENA = { 0x10310001, 1, {} };   // L (1 byte) GPS L1C/A enable
inline UBXParameter SBAS_ENA = { 0x10310020, 1, {} };       // L (1 byte) SBAS enable
inline UBXParameter SBAS_L1CA_ENA = { 0x10310005, 1, {} };  // L (1 byte) SBAS L1C/A enable
inline UBXParameter GAL_ENA = { 0x10310021, 1, {} };        // L (1 byte) Galileo enable
inline UBXParameter GAL_E1_ENA = { 0x10310007, 1, {} };     // L (1 byte) Galileo E1 enable
inline UBXParameter BDS_ENA = { 0x10310022, 1, {} };        // L (1 byte) BeiDou enable
inline UBXParameter BDS_B1_ENA = { 0x1031000D, 1, {} };     // L (1 byte) BeiDou B1I enable
inline UBXParameter BDS_B1C_ENA = { 0x1031000F, 1, {} };    // L (1 byte) BeiDou B1C enable
inline UBXParameter QZSS_ENA = { 0x10310024, 1, {} };       // L (1 byte) QZSS enable
inline UBXParameter QZSS_L1CA_ENA = { 0x10310012, 1, {} };  // L (1 byte) QZSS L1C/A enable
inline UBXParameter QZSS_L1S_ENA = { 0x10310014, 1, {} };   // L (1 byte) QZSS L1S enable
inline UBXParameter GLO_ENA = { 0x10310025, 1, {} };        // L (1 byte) GLONASS enable
inline UBXParameter GLO_L1_ENA = { 0x10310018, 1, {} };     // L (1 byte) GLONASS L1 enable
}


namespace ODO {
// Configuration items for Odometer
inline UBXParameter USE_ODO = { 0x10220001, 1, {} };        // Use odometer (1 byte, L)
inline UBXParameter USE_COG = { 0x10220002, 1, {} };        // Use low-speed course over ground filter (1 byte, L)
inline UBXParameter OUTLPVEL = { 0x10220003, 1, {} };       // Output low-pass filtered velocity (1 byte, L)
inline UBXParameter OUTLPCOG = { 0x10220004, 1, {} };       // Output low-pass filtered course over ground (1 byte, L)
inline UBXParameter PROFILE = { 0x20220005, 1, {} };        // Odometer profile configuration (1 byte, E1)
inline UBXParameter COG_MAXSPEED = { 0x20220021, 1, {} };   // Max speed for course over ground filter (1 byte, U1)
inline UBXParameter COG_MAXPOSACC = { 0x20220022, 1, {} };  // Max position accuracy for course over ground (1 byte, U1)
inline UBXParameter VEL_LPGAIN = { 0x20220031, 1, {} };     // Velocity low-pass filter level (1 byte, U1, 0-255)
inline UBXParameter COG_LPGAIN = { 0x20220032, 1, {} };  // Course over ground low-pass filter level (1 byte, U1, 0-255)

// Constants for CFG-ODO-PROFILE
enum Profile : uint8_t {
	RUN = 0,    // Running
	CYCL = 1,   // Cycling
	SWIM = 2,   // Swimming
	CAR = 3,    // Car
	CUSTOM = 4  // Custom
};

}  // namespace ODO

namespace TP {
inline UBXParameter PULSE_DEF = { 0x20050023, 1, {} };         // Determines whether time pulse is frequency or period
inline UBXParameter PULSE_LENGTH_DEF = { 0x20050030, 1, {} };  // Determines whether pulse length is in [us] or [%]
inline UBXParameter ANT_CABLEDELAY = { 0x30050001, 2, {} };    // Antenna cable delay in [ns]
inline UBXParameter PERIOD_TP1 = { 0x40050002, 4, {} };        // Time pulse period (TP1) in [us]
inline UBXParameter PERIOD_LOCK_TP1 = { 0x40050003, 4, {} };   // Locked time pulse period (TP1) in [us]
inline UBXParameter FREQ_TP1 = { 0x40050024, 4, {} };          // Time pulse frequency (TP1) in [Hz]
inline UBXParameter FREQ_LOCK_TP1 = { 0x40050025, 4, {} };     // Locked time pulse frequency (TP1) in [Hz]
inline UBXParameter LEN_TP1 = { 0x40050004, 4, {} };           // Time pulse length (TP1) in [us]
inline UBXParameter LEN_LOCK_TP1 = { 0x40050005, 4, {} };      // Locked time pulse length (TP1) in [us]
inline UBXParameter DUTY_TP1 = { 0x5005002a, 8, {} };          // Duty cycle of time pulse (TP1) in [%]
inline UBXParameter DUTY_LOCK_TP1 = { 0x5005002b, 8, {} };     // Locked duty cycle of time pulse (TP1) in [%]
inline UBXParameter USER_DELAY_TP1 = { 0x40050006, 4, {} };    // User-defined time pulse delay (TP1) in [ns]
inline UBXParameter TP1_ENA = { 0x10050007, 1, {} };           // Enable the first time pulse
inline UBXParameter SYNC_GNSS_TP1 = { 0x10050008, 1, {} };     // Sync time pulse to GNSS time or local clock (TP1)
inline UBXParameter USE_LOCKED_TP1 = { 0x10050009, 1, {} };    // Use locked parameters if GNSS time is valid (TP1)
inline UBXParameter ALIGN_TO_TOW_TP1 = { 0x1005000a, 1, {} };  // Align time pulse to top of second (TP1)
inline UBXParameter POL_TP1 = { 0x1005000b, 1, {} };           // Time pulse polarity (TP1)
inline UBXParameter TIMEGRID_TP1 = { 0x2005000c, 1, {} };      // Time grid for time pulse (TP1)

// Enums for configuration constants

// Enum for CFG-TP-PULSE_DEF
enum PULSE_DEF {
	PERIOD = 0,  // Time pulse period [us]
	FREQ = 1     // Time pulse frequency [Hz]
};

// Enum for CFG-TP-PULSE_LENGTH_DEF
enum PULSE_LENGTH_DEF {
	RATIO = 0,  // Time pulse ratio
	LENGTH = 1  // Time pulse length [us]
};

// Enum for CFG-TP-TIMEGRID_TP1
enum TIMEGRID_TP1 {
	UTC = 0,    // UTC time reference
	GPS = 1,    // GPS time reference
	GLO = 2,    // GLONASS time reference
	BDS = 3,    // BeiDou time reference
	GAL = 4,    // Galileo time reference
	NAVIC = 5,  // NavIC time reference
	LOCAL = 15  // Receiver's local time reference
};

}  // namespace TP

namespace PM {
inline UBXParameter OPERATEMODE = { 0x20d00001, 1, {} };       // General mode of operation
inline UBXParameter POSUPDATEPERIOD = { 0x40d00002, 4, {} };   // Position update period for PSMOO [s]
inline UBXParameter ACQPERIOD = { 0x40d00003, 4, {} };         // Acquisition period after failed fix [s]
inline UBXParameter GRIDOFFSET = { 0x40d00004, 4, {} };        // Position update period grid offset [s]
inline UBXParameter ONTIME = { 0x30d00005, 2, {} };            // Time to stay in Tracking state [s]
inline UBXParameter MINACQTIME = { 0x20d00006, 1, {} };        // Minimum time in Acquisition state [s]
inline UBXParameter MAXACQTIME = { 0x20d00007, 1, {} };        // Maximum time in Acquisition state [s]
inline UBXParameter DONOTENTEROFF = { 0x10d00008, 1, {} };     // Prevent entering off mode if fix fails
inline UBXParameter WAITTIMEFIX = { 0x10d00009, 1, {} };       // Wait for time fix
inline UBXParameter UPDATEEPH = { 0x10d0000a, 1, {} };         // Update ephemeris regularly
inline UBXParameter EXTINTWAKE = { 0x10d0000c, 1, {} };        // EXTINT pin wake control
inline UBXParameter EXTINTBACKUP = { 0x10d0000d, 1, {} };      // EXTINT pin backup control
inline UBXParameter EXTINTINACTIVE = { 0x10d0000e, 1, {} };    // EXTINT pin inactivity control
inline UBXParameter EXTINTINACTIVITY = { 0x40d0000f, 4, {} };  // EXTINT pin inactivity timeout [ms]
inline UBXParameter LIMITPEAKCURR = { 0x10d00010, 1, {} };     // Limit peak current

// Enums for operation modes
enum OPERATEMODE_VALUES {
	FULL = 0,   // Normal operation, no power save mode
	PSMOO = 1,  // Power save ON/OFF mode
	PSMCT = 2   // Power save cyclic tracking mode
};
}  // namespace PM

namespace CFG {
enum ClearMask : uint32_t {
	CLEARMASK_IOPORT = (1 << 0),
	CLEARMASK_MSGCONF = (1 << 1),
	CLEARMASK_INFMSG = (1 << 2),
	CLEARMASK_NAVCONF = (1 << 3),
	CLEARMASK_RXMCONF = (1 << 4),
	CLEARMASK_SENCONF = (1 << 8),
	CLEARMASK_RINVCONF = (1 << 9),
	CLEARMASK_ANTCONF = (1 << 10),
	CLEARMASK_LOGCONF = (1 << 11),
	CLEARMASK_FTSCONF = (1 << 12)
};

enum DevMask : uint8_t {
	DEVMASK_BBR = (1 << 0),
	DEVMASK_FLASH = (1 << 1),
	DEVMASK_EEPROM = (1 << 2),
	DEVMASK_SPIFLASH = (1 << 4)
};

struct __attribute__((__packed__)) MSG_CFG {
	uint32_t clearMask;
	uint32_t saveMask;
	uint32_t loadMask;
	uint8_t deviceMask;
};
}  // namespace CFG


namespace NAVSPG {
// Parameters
inline UBXParameter FIXMODE = { 0x20110011, 1, {} };         // Position fix mode
inline UBXParameter INIFIX3D = { 0x10110013, 1, {} };        // Initial fix must be a 3D fix
inline UBXParameter WKNROLLOVER = { 0x30110017, 2, {} };     // GPS week rollover number
inline UBXParameter UTCSTANDARD = { 0x2011001c, 1, {} };     // UTC standard
inline UBXParameter DYNMODEL = { 0x20110021, 1, {} };        // Dynamic platform model
inline UBXParameter ACKAIDING = { 0x10110025, 1, {} };       // Acknowledge assistance input messages
inline UBXParameter USE_USRDAT = { 0x10110061, 1, {} };      // Use user geodetic datum parameters
inline UBXParameter USRDAT_MAJA = { 0x50110062, 8, {} };     // Geodetic datum semi-major axis [m]
inline UBXParameter USRDAT_FLAT = { 0x50110063, 8, {} };     // Geodetic datum 1.0 / flattening
inline UBXParameter USRDAT_DX = { 0x40110064, 4, {} };       // Geodetic datum X axis shift [m]
inline UBXParameter USRDAT_DY = { 0x40110065, 4, {} };       // Geodetic datum Y axis shift [m]
inline UBXParameter USRDAT_DZ = { 0x40110066, 4, {} };       // Geodetic datum Z axis shift [m]
inline UBXParameter USRDAT_ROTX = { 0x40110067, 4, {} };     // Geodetic datum rotation X axis [arcsec]
inline UBXParameter USRDAT_ROTY = { 0x40110068, 4, {} };     // Geodetic datum rotation Y axis [arcsec]
inline UBXParameter USRDAT_ROTZ = { 0x40110069, 4, {} };     // Geodetic datum rotation Z axis [arcsec]
inline UBXParameter USRDAT_SCALE = { 0x4011006a, 4, {} };    // Geodetic datum scale factor [ppm]
inline UBXParameter INFIL_MINSVS = { 0x201100a1, 1, {} };    // Minimum number of satellites for navigation
inline UBXParameter INFIL_MAXSVS = { 0x201100a2, 1, {} };    // Maximum number of satellites for navigation
inline UBXParameter INFIL_MINCNO = { 0x201100a3, 1, {} };    // Minimum satellite signal level for navigation [dBHz]
inline UBXParameter INFIL_MINELEV = { 0x201100a4, 1, {} };   // Minimum elevation for GNSS satellite [deg]
inline UBXParameter INFIL_NCNOTHRS = { 0x201100aa, 1, {} };  // Required satellites above C/N0 threshold
inline UBXParameter INFIL_CNOTHRS = { 0x201100ab, 1, {} };   // C/N0 threshold for fix attempt
inline UBXParameter OUTFIL_PDOP = { 0x301100b1, 2, {} };     // Output filter position DOP threshold [0.1]
inline UBXParameter OUTFIL_TDOP = { 0x301100b2, 2, {} };     // Output filter time DOP threshold [0.1]
inline UBXParameter OUTFIL_PACC = { 0x301100b3, 2, {} };     // Output filter position accuracy mask [m]
inline UBXParameter OUTFIL_TACC = { 0x301100b4, 2, {} };     // Output filter time accuracy mask [m]
inline UBXParameter OUTFIL_FACC = { 0x301100b5, 2, {} };     // Output filter frequency accuracy mask [m/s]
inline UBXParameter CONSTR_ALT = { 0x401100c1, 4, {} };      // Fixed altitude for 2D fix mode [m]
inline UBXParameter CONSTR_ALTVAR = { 0x401100c2, 4, {} };   // Fixed altitude variance for 2D mode [m^2]
inline UBXParameter CONSTR_DGNSSTO = { 0x201100c4, 1, {} };  // DGNSS timeout [s]
inline UBXParameter SIGATTCOMP = { 0x201100d6, 1, {} };      // Permanently attenuated signal compensation mode

// Enums for constants

// Enum for CFG-NAVSPG-FIXMODE
enum FIXMODE_VALUES : uint8_t {
	FIXMODE_2DONLY = 1,  // 2D only
	FIXMODE_3DONLY = 2,  // 3D only
	FIXMODE_AUTO = 3     // Auto 2D/3D
};

// Enum for CFG-NAVSPG-UTCSTANDARD
enum UTCSTANDARD_VALUES : uint8_t {
	UTC_AUTO = 0,
	UTC_USNO = 3,  // USNO (GPS time)
	UTC_EU = 5,    // European (Galileo time)
	UTC_SU = 6,    // Soviet Union (GLONASS time)
	UTC_NTSC = 7,  // NTSC China (BeiDou time)
	UTC_NPLI = 8,  // NPLI India (NavIC time)
	UTC_NICT = 9   // NICT Japan (QZSS time)
};

// Enum for CFG-NAVSPG-DYNMODEL
enum DYNMODEL_VALUES : uint8_t {
	MODEL_PORTABLE = 0,    // Portable
	MODEL_STATIONARY = 2,  // Stationary
	MODEL_PEDESTRIAN = 3,  // Pedestrian
	MODEL_AUTOMOTIVE = 4,  // Automotive
	MODEL_SEA = 5,         // Sea
	MODEL_AIR1 = 6,        // Airborne <1g
	MODEL_AIR2 = 7,        // Airborne <2g
	MODEL_AIR4 = 8,        // Airborne <4g
	MODEL_WRIST = 9,       // Wrist-worn watch (limited)
	MODEL_BIKE = 10,       // Motorbike (limited)
	MODEL_MOWER = 11,      // Robotic lawn mower (limited)
	MODEL_ESCOOTER = 12    // E-scooter (limited)
};

// Enum for CFG-NAVSPG-SIGATTCOMP
enum SIGATTCOMP_VALUES : uint8_t {
	SIGCOMP_DISABLE = 0,
	SIGCOMP_01DBHZ = 1,
	SIGCOMP_02DBHZ = 2,
	SIGCOMP_03DBHZ = 3,
	SIGCOMP_04DBHZ = 4,
	SIGCOMP_05DBHZ = 5,
	SIGCOMP_06DBHZ = 6,
	SIGCOMP_07DBHZ = 7,
	SIGCOMP_08DBHZ = 8,
	SIGCOMP_09DBHZ = 9,
	SIGCOMP_10DBHZ = 10,
	SIGCOMP_11DBHZ = 11,
	SIGCOMP_12DBHZ = 12,
	SIGCOMP_13DBHZ = 13,
	SIGCOMP_14DBHZ = 14,
	SIGCOMP_15DBHZ = 15,
	SIGCOMP_16DBHZ = 16,
	SIGCOMP_17DBHZ = 17,
	SIGCOMP_18DBHZ = 18,
	SIGCOMP_19DBHZ = 19,
	SIGCOMP_20DBHZ = 20,
	SIGCOMP_21DBHZ = 21,
	SIGCOMP_22DBHZ = 22,
	SIGCOMP_23DBHZ = 23,
	SIGCOMP_24DBHZ = 24,
	SIGCOMP_25DBHZ = 25,
	SIGCOMP_26DBHZ = 26,
	SIGCOMP_27DBHZ = 27,
	SIGCOMP_28DBHZ = 28,
	SIGCOMP_29DBHZ = 29,
	SIGCOMP_30DBHZ = 30,
	SIGCOMP_31DBHZ = 31,
	SIGCOMP_32DBHZ = 32,
	SIGCOMP_33DBHZ = 33,
	SIGCOMP_34DBHZ = 34,
	SIGCOMP_35DBHZ = 35,
	SIGCOMP_36DBHZ = 36,
	SIGCOMP_37DBHZ = 37,
	SIGCOMP_38DBHZ = 38,
	SIGCOMP_39DBHZ = 39,
	SIGCOMP_40DBHZ = 40,
	SIGCOMP_41DBHZ = 41,
	SIGCOMP_42DBHZ = 42,
	SIGCOMP_43DBHZ = 43,
	SIGCOMP_44DBHZ = 44,
	SIGCOMP_45DBHZ = 45,
	SIGCOMP_46DBHZ = 46,
	SIGCOMP_47DBHZ = 47,
	SIGCOMP_48DBHZ = 48,
	SIGCOMP_49DBHZ = 49,
	SIGCOMP_50DBHZ = 50,
	SIGCOMP_51DBHZ = 51,
	SIGCOMP_52DBHZ = 52,
	SIGCOMP_53DBHZ = 53,
	SIGCOMP_54DBHZ = 54,
	SIGCOMP_55DBHZ = 55,
	SIGCOMP_56DBHZ = 56,
	SIGCOMP_57DBHZ = 57,
	SIGCOMP_58DBHZ = 58,
	SIGCOMP_59DBHZ = 59,
	SIGCOMP_60DBHZ = 60,
	SIGCOMP_61DBHZ = 61,
	SIGCOMP_62DBHZ = 62,
	SIGCOMP_63DBHZ = 63,
	SIGCOMP_AUTO = 255
};

}

namespace ANA {
inline UBXParameter USE_ANA = { 0x10230001, 1, {} };    // Use AssistNow Autonomous
inline UBXParameter ORBMAXERR = { 0x30230002, 2, {} };  // Maximum acceptable modeled orbit error in meters

// Enum for AssistNow Autonomous use
enum USE_ANA_VALUES { ANA_DISABLED = 0, ANA_ENABLED = 1 };
}  // namespace ANA

/// @brief Every CFG parameter this firmware knows, as pointers to the single
/// declaration of each. Pointers, not copied {key, size} pairs: an entry cannot
/// disagree with the parameter it names, so the table cannot drift from the
/// declarations above it, and adding a parameter is one line here.
///
/// This replaced a chain of 111 `if (key == X.key) return X.size;`. That chain
/// had already drifted: RXM_MEASC12_UART1, RXM_MEAS20_UART1 and RXM_MEAS50_UART1
/// were declared and are actively written by the CloudLocate configuration
/// (m10qasync_config.cpp:722/735/748) but were missing from it, so
/// getParameterSize() returned 0 for them and the VALGET parser treated the
/// whole rest of the response as unreadable and stopped. Nobody adds a line to a
/// 111-line if-chain reliably; a table is read as data, and a missing entry is
/// visible next to its neighbours.
static const UBXParameter *const PARAMETERS[] = {
	// UART1
	&UART1::BAUDRATE,
	&UART1::STOPBITS,
	&UART1::DATABITS,
	&UART1::PARITY,
	&UART1::ENABLED,
	&UART1::INPROT_UBX,
	&UART1::INPROT_NMEA,
	&UART1::OUTPROT_UBX,
	&UART1::OUTPROT_NMEA,
	&UART1::TXREADY_ENABLED,
	&UART1::TXREADY_POLARITY,
	&UART1::TXREADY_PIN,
	&UART1::TXREADY_THRESHOLD,
	&UART1::TXREADY_INTERFACE,
	// MSG
	&MSG::MSG_CLASS,
	&MSG::MSG_ID,
	&MSG::RATE,
	// MSGOUT
	&MSGOUT::NAV_PVT_UART1,
	&MSGOUT::NAV_DOP_UART1,
	&MSGOUT::NAV_STATUS_UART1,
	&MSGOUT::NAV_SAT_UART1,
	&MSGOUT::RXM_MEASC12_UART1,
	&MSGOUT::RXM_MEAS20_UART1,
	&MSGOUT::RXM_MEAS50_UART1,
	// SIGNAL
	&SIGNAL::GPS_ENA,
	&SIGNAL::GPS_L1CA_ENA,
	&SIGNAL::SBAS_ENA,
	&SIGNAL::SBAS_L1CA_ENA,
	&SIGNAL::GAL_ENA,
	&SIGNAL::GAL_E1_ENA,
	&SIGNAL::BDS_ENA,
	&SIGNAL::BDS_B1_ENA,
	&SIGNAL::BDS_B1C_ENA,
	&SIGNAL::QZSS_ENA,
	&SIGNAL::QZSS_L1CA_ENA,
	&SIGNAL::QZSS_L1S_ENA,
	&SIGNAL::GLO_ENA,
	&SIGNAL::GLO_L1_ENA,
	// ODO
	&ODO::USE_ODO,
	&ODO::USE_COG,
	&ODO::OUTLPVEL,
	&ODO::OUTLPCOG,
	&ODO::PROFILE,
	&ODO::COG_MAXSPEED,
	&ODO::COG_MAXPOSACC,
	&ODO::VEL_LPGAIN,
	&ODO::COG_LPGAIN,
	// TP
	&TP::PULSE_DEF,
	&TP::PULSE_LENGTH_DEF,
	&TP::ANT_CABLEDELAY,
	&TP::PERIOD_TP1,
	&TP::PERIOD_LOCK_TP1,
	&TP::FREQ_TP1,
	&TP::FREQ_LOCK_TP1,
	&TP::LEN_TP1,
	&TP::LEN_LOCK_TP1,
	&TP::DUTY_TP1,
	&TP::DUTY_LOCK_TP1,
	&TP::USER_DELAY_TP1,
	&TP::TP1_ENA,
	&TP::SYNC_GNSS_TP1,
	&TP::USE_LOCKED_TP1,
	&TP::ALIGN_TO_TOW_TP1,
	&TP::POL_TP1,
	&TP::TIMEGRID_TP1,
	// PM
	&PM::OPERATEMODE,
	&PM::POSUPDATEPERIOD,
	&PM::ACQPERIOD,
	&PM::GRIDOFFSET,
	&PM::ONTIME,
	&PM::MINACQTIME,
	&PM::MAXACQTIME,
	&PM::DONOTENTEROFF,
	&PM::WAITTIMEFIX,
	&PM::UPDATEEPH,
	&PM::EXTINTWAKE,
	&PM::EXTINTBACKUP,
	&PM::EXTINTINACTIVE,
	&PM::EXTINTINACTIVITY,
	&PM::LIMITPEAKCURR,
	// NAVSPG
	&NAVSPG::FIXMODE,
	&NAVSPG::INIFIX3D,
	&NAVSPG::WKNROLLOVER,
	&NAVSPG::UTCSTANDARD,
	&NAVSPG::DYNMODEL,
	&NAVSPG::ACKAIDING,
	&NAVSPG::USE_USRDAT,
	&NAVSPG::USRDAT_MAJA,
	&NAVSPG::USRDAT_FLAT,
	&NAVSPG::USRDAT_DX,
	&NAVSPG::USRDAT_DY,
	&NAVSPG::USRDAT_DZ,
	&NAVSPG::USRDAT_ROTX,
	&NAVSPG::USRDAT_ROTY,
	&NAVSPG::USRDAT_ROTZ,
	&NAVSPG::USRDAT_SCALE,
	&NAVSPG::INFIL_MINSVS,
	&NAVSPG::INFIL_MAXSVS,
	&NAVSPG::INFIL_MINCNO,
	&NAVSPG::INFIL_MINELEV,
	&NAVSPG::INFIL_NCNOTHRS,
	&NAVSPG::INFIL_CNOTHRS,
	&NAVSPG::OUTFIL_PDOP,
	&NAVSPG::OUTFIL_TDOP,
	&NAVSPG::OUTFIL_PACC,
	&NAVSPG::OUTFIL_TACC,
	&NAVSPG::OUTFIL_FACC,
	&NAVSPG::CONSTR_ALT,
	&NAVSPG::CONSTR_ALTVAR,
	&NAVSPG::CONSTR_DGNSSTO,
	&NAVSPG::SIGATTCOMP,
	// ANA
	&ANA::USE_ANA,
	&ANA::ORBMAXERR,
};

/// @brief Storage size of a CFG key, or 0 if this firmware does not know the key.
///
/// Returning 0 for an unknown key is deliberate and unchanged: the keys reach
/// here from the module's own VALGET reply bytes, not from the declarations, so a
/// key the firmware never declared is reachable and the caller needs to be told.
///
/// NOTE on a tempting shortcut: the key encodes its own storage size. This is
/// specified, not inferred -- u-blox M10 SPG 5.10 Interface description
/// UBX-21035062 R03, section 4.2: "Bits 30...28: Three bits that indicate the
/// storage size of a Configuration Value (range 0x01-0x05)", where 0x01 is one
/// bit (stored in a byte), 0x02 one byte, 0x03 two bytes, 0x04 four bytes and
/// 0x05 eight bytes. It also holds for all 113 declarations here with no
/// exception, which the test suite asserts.
///
/// Deriving the size from the key would make this function three lines, and it
/// is still the wrong trade: it would answer for keys this firmware has never
/// heard of, and that 0 is precisely the signal the caller uses to stop reading
/// a reply it cannot interpret. The table keeps that answer. The spec decides
/// the size of a key; only this table decides which keys we claim to know.
static uint8_t getParameterSize(uint32_t key) {
	for (const UBXParameter *p : PARAMETERS) {
		if (p->key == key) return p->size;
	}
	return 0;  // Return 0 for unknown keys
}


}  // namespace CFG

/****************************** NAV *************************************/

namespace NAV {
enum Id : uint8_t {
	ID_AOPSTATUS = 0x60,
	ID_CLOCK = 0x22,
	ID_DGPS = 0x31,
	ID_DOP = 0x04,
	ID_POSECEF = 0x01,
	ID_POSLLH = 0x02,
	ID_PVT = 0x07,
	ID_SBAS = 0x32,
	ID_ORB = 0x34,
	ID_SAT = 0x35,
	ID_SOL = 0x06,
	ID_STATUS = 0x03,
	ID_SVINFO = 0x30,
	ID_TIMEGPS = 0x20,
	ID_TIMEUTC = 0x21,
	ID_VELECEF = 0x11,
	ID_VELNED = 0x12
};

namespace AOPSTATUS {
enum AopCfg : uint8_t { AOPCFG_AOP_DISABLED = 0, AOPCFG_AOP_ENABLED = 1 << 0 };

struct __attribute__((__packed__)) MSG_AOPSTATUS {
	uint32_t iTow;
	AopCfg aopCfg;
	uint8_t status;
	uint8_t reserved1[10];
};
}  // namespace AOPSTATUS

namespace DOP {
struct __attribute__((__packed__)) MSG_DOP {
	uint32_t iTow;
	uint16_t gDOP;
	uint16_t pDOP;
	uint16_t tDOP;
	uint16_t vDOP;
	uint16_t hDOP;
	uint16_t nDOP;
	uint16_t eDOP;
};
}  // namespace DOP

namespace STATUS {
struct __attribute__((__packed__)) MSG_STATUS {
	uint32_t iTow;
	uint8_t gpsFix;
	uint8_t flags;
	uint8_t fixStat;
	uint8_t flags2;
	uint32_t ttff;
	uint32_t msss;
};
}  // namespace STATUS

namespace PVT {
enum FixType : uint8_t {
	FIXTYPE_NO = 0,
	FIXTYPE_DEAD_RECKONING_ONLY = 1,
	FIXTYPE_2D = 2,
	FIXTYPE_3D = 3,
	FIXTYPE_GNSS_AND_DEAD_RECKONING = 4,
	FIXTYPE_TIME_ONLY = 5,
};

enum VALID : uint8_t {
	VALID_VALID_DATE = 1 << 0,
	VALID_VALID_TIME = 1 << 1,
	VALID_FULLY_RESOLVED = 1 << 2,
	VALID_MAG = 1 << 3,
};

struct __attribute__((__packed__)) MSG_PVT {
	uint32_t iTow;  // GPS time of week of the navigation epoch
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t valid;  // Validity flags
	uint32_t tAcc;  // Time accuracy estimate in nanoseconds
	int32_t nano;
	FixType fixType;   // GNSSfix Type
	uint8_t flags;     // Fix status flags
	uint8_t flags2;    // Additional flags
	uint8_t numSV;     // Number of satellites used in Nav Solution
	int32_t lon;       // Longitude
	int32_t lat;       // Latitude
	int32_t height;    // Height above ellipsoid
	int32_t hMSL;      // Height above mean sea level
	uint32_t hAcc;     // Horizontal accuracy estimate
	uint32_t vAcc;     // Vertical accuracy estimate
	int32_t velN;      // NED north velocity
	int32_t velE;      // NED east velocity
	int32_t velD;      // NED down velocity
	int32_t gSpeed;    // Ground speed (2D)
	int32_t headMot;   // Heading of motion (2D)
	uint32_t sAcc;     // Speed accuracy estimate
	uint32_t headAcc;  // Heading accuracy estimate (both motion and  vehicle)
	uint16_t pDOP;     // Position DOP
	uint8_t flags3;    // Additional flags
	uint8_t reserved1[5];
	int32_t headVeh;  // Heading of vehicle (2D)
	int16_t magDec;   // Magnetic declination
	uint16_t magAcc;  // Magnetic declination accuracy
};
}  // namespace PVT

namespace SAT {
struct __attribute__((__packed__)) MSG_SAT {
	uint32_t iTow;
	uint8_t version;
	uint8_t numSvs;
	uint8_t reserved[2];
	struct __attribute__((__packed__)) {
		uint8_t gnssId;
		uint8_t svId;
		uint8_t cno;
		int8_t elev;
		int16_t azim;
		int16_t prRes;
		uint32_t qualityInd : 3;
		uint32_t svUsed : 1;
		uint32_t health : 2;
		uint32_t diffCorr : 1;
		uint32_t smoothed : 1;
		uint32_t orbitSource : 3;
		uint32_t ephAvail : 1;
		uint32_t almAvail : 1;
		uint32_t anoAvail : 1;
		uint32_t aopAvail : 1;
		uint32_t sbasCorrUsed : 1;
		uint32_t rtcmCorrUsed : 1;
		uint32_t slasCorrUsed : 1;
		uint32_t sparnCorrUsed : 1;
		uint32_t prCorrUsed : 1;
		uint32_t crCorrUsed : 1;
		uint32_t doCorrUsed : 1;
		uint32_t unused : 10;
	} svInfo[12];
};
}
}  // namespace NAV

/****************************** MGA *************************************/

namespace RXM {
enum Id : uint8_t {
	ID_PMREQ = 0x41,
	ID_MEAS20 = 0x84,  // RXM-MEAS20: 20-byte compact raw GNSS measurement (CloudLocate)
	ID_MEAS50 = 0x86,  // RXM-MEAS50: 50-byte extended raw GNSS measurement (CloudLocate)
	ID_MEASC12 = 0x82  // RXM-MEASC12: 12-byte minimal raw GNSS measurement (CloudLocate)
};

enum PMREQFlags : uint8_t { BACKUP = 2, FORCE = 4 };

enum PMREQWakeupSources : uint8_t { UARTRX = 8, EXTINT0 = 32, EXTINT1 = 64, SPICS = 128 };

struct __attribute__((__packed__)) MSG_PMREQ {
	uint8_t version;
	uint8_t reserved1[3];
	uint32_t duration;
	uint32_t flags;
	uint32_t wakeupSources;
};

// CloudLocate raw measurement blobs (opaque, forwarded as-is to u-blox cloud)
struct __attribute__((__packed__)) MSG_MEASC12 {
	uint8_t data[12];
};
struct __attribute__((__packed__)) MSG_MEAS20 {
	uint8_t data[20];
};
struct __attribute__((__packed__)) MSG_MEAS50 {
	uint8_t data[50];
};
}

namespace MGA {
enum Id : uint8_t {
	ID_INI = 0x40,           // MGA-INI (type field selects TIME_UTC=0x10 or POS_LLH=0x01)
	ID_INI_TIME_UTC = 0x40,  // Alias for backwards compatibility
	ID_ANO = 0x20,
	ID_ACK = 0x60,
	ID_DBD = 0x80
};

struct __attribute__((__packed__)) MSG_DBD {};

struct __attribute__((__packed__)) MSG_ANO {
	uint8_t type;
	uint8_t version;
	uint8_t svId;
	uint8_t gnssId;
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t reserved1;
	uint8_t data[64];
	uint8_t reserved2[4];
};

struct __attribute__((__packed__)) MSG_ACK {
	uint8_t type;
	uint8_t version;
	uint8_t infoCode;
	uint8_t msgId;
	uint32_t msgPayloadStart;
};

struct __attribute__((__packed__)) MSG_INI_TIME_UTC {
	uint8_t type;
	uint8_t version;
	uint8_t ref;
	int8_t leapSecs;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t reserved1;
	uint32_t ns;
	uint16_t tAccS;
	uint8_t reserved2[2];
	uint32_t tAccNs;
};

// MGA-INI-POS_LLH: Initial position assistance (type=0x01)
struct __attribute__((__packed__)) MSG_INI_POS_LLH {
	uint8_t type;     // 0x01 for position
	uint8_t version;  // 0x00
	uint8_t reserved1[2];
	int32_t lat;      // Latitude [deg * 1e-7]
	int32_t lon;      // Longitude [deg * 1e-7]
	int32_t alt;      // Altitude [cm]
	uint32_t posAcc;  // Position accuracy [cm]
};
}  // namespace ACK
}  // namespace UBX
