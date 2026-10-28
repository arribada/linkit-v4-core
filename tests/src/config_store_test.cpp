#include "config_store_fs.hpp"

#include "dte_protocol.hpp"
#include "fake_battery_mon.hpp"
#include "calibration.hpp"
#include "hauled_mode_service.hpp"
#include "moored_mode_service.hpp"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

using namespace std::string_literals;

#define BLOCK_COUNT   (256)
#define BLOCK_SIZE    (64 * 1024)
#define PAGE_SIZE     (256)
#define MAX_FILE_SIZE (4 * 1024 * 1024)


extern FileSystem *main_filesystem;
extern BatteryMonitor *battery_monitor;
extern ConfigurationStore *configuration_store;
static LFSFileSystem *ram_filesystem;

using namespace std::literals::string_literals;


TEST_GROUP(ConfigStore) {
	RamFlash *ram_flash;
	FakeBatteryMonitor *fake_battery_monitor;
	LFSConfigurationStore *store = nullptr;

	void setup() {
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		ram_flash = new RamFlash(BLOCK_COUNT, BLOCK_SIZE, PAGE_SIZE);
		ram_filesystem = new LFSFileSystem(ram_flash);
		ram_filesystem->format();
		ram_filesystem->mount();
		main_filesystem = ram_filesystem;
	}

	void teardown() {
		// The mode classifiers are static and read the GLOBAL configuration
		// store. Leaving either behind would leak a moored/hauled state — and a
		// dangling store pointer — into whichever group runs next, where it
		// would quietly rewrite the GNSS period and TX interval.
		MooredModeService::reset_for_tests();
		HauledModeService::reset_for_tests();
		configuration_store = nullptr;
		delete store;
		store = nullptr;
		ram_filesystem->umount();
		delete ram_filesystem;
		main_filesystem = nullptr;
		delete ram_flash;
		delete fake_battery_monitor;
	}
};


TEST(ConfigStore, CreateConfigStoreWithDefaultParams) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();
	CHECK_TRUE(store->is_valid());

	// Check some defaults are correct
	CHECK_EQUAL(0U, store->read_param<unsigned int>(ParamID::ARGOS_DECID));
	CHECK_EQUAL(DEVICE_MODEL_NAME, store->read_param<std::string>(ParamID::DEVICE_MODEL));
	CHECK_EQUAL(0U, (unsigned int)store->read_param<BaseUnderwaterDetectSource>(ParamID::UNDERWATER_DETECT_SOURCE));
	CHECK_EQUAL(1.1, store->read_param<double>(ParamID::UNDERWATER_DETECT_THRESH));
}

TEST(ConfigStore, CheckBaseTypeReadAccess) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Modify some parameter values
	std::string model = "GenTracker";
	store->write_param(ParamID::PROFILE_NAME, model);

	BaseType x = store->read_param<BaseType>(ParamID::PROFILE_NAME);
	CHECK_EQUAL(model, std::get<std::string>(x));
}

TEST(ConfigStore, CheckConfigStorePersistence) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	// Modify some parameter values
	std::string model = "GenTracker";
	unsigned int dec_id = 1234U;
	store->write_param(ParamID::ARGOS_DECID, dec_id);
	store->write_param(ParamID::PROFILE_NAME, model);
	store->save_params();

	// Delete the object and recreate a new one
	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();  // This will read in the saved file

	// Check modified parameters
	CHECK_EQUAL(1234U, store->read_param<unsigned int>(ParamID::ARGOS_DECID));
	CHECK_EQUAL(model, store->read_param<std::string>(ParamID::PROFILE_NAME));
}

// A configuration version bump must not destroy the credentials.
//
// config_store_fs.hpp declares PROTECTED_PARAMS, thirteen entries that are ALL
// the credentials: the Argos secret key, the four RADIOCONF, and the eight
// LoRaWAN keys. config_store.hpp:159-195 spells out what losing them costs --
// "would leave every provisioned LoRa unit unable to rejoin its network, with
// nothing in the logs to say why" -- and that is why the version code is
// deliberately not bumped.
//
// This pins the recovery itself. The version-mismatch path reads records
// POSITIONALLY, so it recovered exactly the two that happen to sit first in
// param_map (ARGOS_DECID index 0, ARGOS_HEXID index 1) and lost the other
// eleven, which are scattered. ARGOS_SECKEY is skipped here because it is
// is_implemented only when ARGOS_SMD == 1, which the host build does not set.
TEST(ConfigStore, VersionMismatchKeepsEveryProtectedParam) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	const std::string radioconf = "RADIOCONF-MASTER";
	const std::string ldk = "RADIOCONF-LDK";
	const std::string lda2 = "RADIOCONF-LDA2";
	const std::string vlda4 = "RADIOCONF-VLDA4";
	const std::string deveui = "70B3D57ED0001234";
	const std::string appeui = "0000000000000001";
	const std::string appkey = "0123456789ABCDEF0123456789ABCDEF";
	const std::string devaddr = "26011BDA";
	const std::string appskey = "FEDCBA9876543210FEDCBA9876543210";
	const std::string nwkskey = "11223344556677889900AABBCCDDEEFF";
	const unsigned int decid = 4321U;

	store->write_param(ParamID::ARGOS_RADIOCONF, radioconf);
	store->write_param(ParamID::ARGOS_RADIOCONF_LDK, ldk);
	store->write_param(ParamID::ARGOS_RADIOCONF_LDA2, lda2);
	store->write_param(ParamID::ARGOS_RADIOCONF_VLDA4, vlda4);
	store->write_param(ParamID::LORA_DEVEUI, deveui);
	store->write_param(ParamID::LORA_APPEUI, appeui);
	store->write_param(ParamID::LORA_APPKEY, appkey);
	store->write_param(ParamID::LORA_DEVADDR, devaddr);
	store->write_param(ParamID::LORA_APPSKEY, appskey);
	store->write_param(ParamID::LORA_NWKSKEY, nwkskey);
	store->write_param(ParamID::ARGOS_DECID, decid);
	store->save_params();

	delete store;
	store = nullptr;

	// Simulate the bump: overwrite only the version field at the head of the
	// file, leaving every record intact. This is exactly what a device sees
	// after a firmware update that changed the version code.
	{
		LFSFile f(main_filesystem, "config.dat", LFS_O_WRONLY);
		unsigned int foreign_version = 0xDEADBEEFU;
		CHECK_EQUAL((lfs_ssize_t)sizeof(foreign_version), f.write(&foreign_version, sizeof(foreign_version)));
	}

	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// ARGOS_DECID already survived before this test existed — it is record 0,
	// which the positional recovery happened to reach. It is asserted so a
	// regression there cannot hide behind the eleven below.
	CHECK_EQUAL(decid, store->read_param<unsigned int>(ParamID::ARGOS_DECID));

	CHECK_EQUAL(radioconf, store->read_param<std::string>(ParamID::ARGOS_RADIOCONF));
	CHECK_EQUAL(ldk, store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDK));
	CHECK_EQUAL(lda2, store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDA2));
	CHECK_EQUAL(vlda4, store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_VLDA4));
	CHECK_EQUAL(deveui, store->read_param<std::string>(ParamID::LORA_DEVEUI));
	CHECK_EQUAL(appeui, store->read_param<std::string>(ParamID::LORA_APPEUI));
	CHECK_EQUAL(appkey, store->read_param<std::string>(ParamID::LORA_APPKEY));
	CHECK_EQUAL(devaddr, store->read_param<std::string>(ParamID::LORA_DEVADDR));
	CHECK_EQUAL(appskey, store->read_param<std::string>(ParamID::LORA_APPSKEY));
	CHECK_EQUAL(nwkskey, store->read_param<std::string>(ParamID::LORA_NWKSKEY));
}

// --- Each record goes to the slot that carries its key ---------------------
//
// config.dat is a version word, then one fixed-size record per slot the
// writing build implements, <KEY><VALUE zero-padded>, in table order. Builds
// that implement different slots write different sequences: the camera slots
// 166-171 exist only with ENABLE_CAM_SENSOR, SECKEY only on SMD, PWP05/LBP14
// only on RSPB since f4ba6c16 (on every board before). deserialize_config()
// used to take the records in order, one per slot, and to drop the slot when
// the key did not match -- consuming the record all the same. One record more
// or less shifted every one after it: they all came back at their defaults,
// and init() saved that for good. On the bench board on 2026-10-27, a camera
// build reflashed without the camera lost 71 parameters from slot 172 on, the
// master RCONF and LAST_KNOWN_RTC among them (clock back to 1970). Every
// v4.1.9 -> v4.1.10 upgrade lost 17 the same way (ARP36/37, see the version
// constant in config_store.hpp).
//
// The helpers below rewrite the file the way another build would have.

static constexpr size_t CONFIG_HEADER = sizeof(unsigned int);
static constexpr size_t CONFIG_RECORD = KEY_LENGTH + BASE_TEXT_MAX_LENGTH;

static std::vector<uint8_t> read_config_file() {
	LFSFile f(main_filesystem, "config.dat", LFS_O_RDONLY);
	std::vector<uint8_t> bytes((size_t)f.size());
	CHECK_EQUAL((lfs_ssize_t)bytes.size(), f.read(bytes.data(), bytes.size()));
	return bytes;
}

static void write_config_file(std::vector<uint8_t> &bytes) {
	LFSFile f(main_filesystem, "config.dat", LFS_O_WRONLY | LFS_O_TRUNC);
	CHECK_EQUAL((lfs_ssize_t)bytes.size(), f.write(bytes.data(), bytes.size()));
}

/// Offset of the record of `id`, or 0 when the file has none.
static size_t config_record_offset(const std::vector<uint8_t> &bytes, ParamID id) {
	const std::string &key = param_map[(unsigned int)id].key;
	for (size_t off = CONFIG_HEADER; off + CONFIG_RECORD <= bytes.size(); off += CONFIG_RECORD)
		if (std::memcmp(&bytes[off], key.data(), KEY_LENGTH) == 0) return off;
	return 0;
}

/// A UINT record as a build that has slot `key` writes it.
static std::vector<uint8_t> uint_config_record(const std::string &key, unsigned int value) {
	std::vector<uint8_t> record(CONFIG_RECORD, 0);
	std::memcpy(record.data(), key.data(), KEY_LENGTH);
	std::memcpy(&record[KEY_LENGTH], &value, sizeof(value));
	return record;
}

/// Values spread over the table, before and after every slot where builds
/// differ, so that a shift anywhere shows.
struct ConfigMarkers {
	unsigned int decid = 4321U;                               // slot 0
	std::string radioconf = "RADIOCONF-MASTER";               // 173
	unsigned int last_known_rtc = 1793120000U;                // 179
	std::string appkey = "0123456789ABCDEF0123456789ABCDEF";  // 183
	unsigned int deep_idle_s = 1800U;                         // 240
	unsigned int log_level = 1U;                              // 282, the last slot

	void write(ConfigurationStore &s) const {
		s.write_param(ParamID::ARGOS_DECID, decid);
		s.write_param(ParamID::ARGOS_RADIOCONF, radioconf);
		s.write_param(ParamID::LAST_KNOWN_RTC, last_known_rtc);
		s.write_param(ParamID::LORA_APPKEY, appkey);
		s.write_param(ParamID::GNSS_DEEP_IDLE_AFTER_OFF_S, deep_idle_s);
		s.write_param(ParamID::SYSTEM_LOG_LEVEL, log_level);
	}
	void check(ConfigurationStore &s) const {
		CHECK_EQUAL(decid, s.read_param<unsigned int>(ParamID::ARGOS_DECID));
		CHECK_EQUAL(radioconf, s.read_param<std::string>(ParamID::ARGOS_RADIOCONF));
		CHECK_EQUAL(last_known_rtc, s.read_param<unsigned int>(ParamID::LAST_KNOWN_RTC));
		CHECK_EQUAL(appkey, s.read_param<std::string>(ParamID::LORA_APPKEY));
		CHECK_EQUAL(deep_idle_s, s.read_param<unsigned int>(ParamID::GNSS_DEEP_IDLE_AFTER_OFF_S));
		CHECK_EQUAL(log_level, s.read_param<unsigned int>(ParamID::SYSTEM_LOG_LEVEL));
	}
};

// The file of a LinkIt build from before f4ba6c16, which still wrote PWP05 and
// LBP14, read by one that no longer has them.
TEST(ConfigStore, RecordsOfSlotsThisBuildLacksShiftNothing) {
	CHECK_FALSE(param_map[(unsigned int)ParamID::SHUTDOWN_NTIME_SAT].is_implemented);
	CHECK_FALSE(param_map[(unsigned int)ParamID::LB_SHUTDOWN_NTIME_SAT].is_implemented);

	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	const ConfigMarkers markers;
	markers.write(*store);
	store->save_params();
	delete store;
	store = nullptr;

	std::vector<uint8_t> bytes = read_config_file();
	const size_t this_build_size = bytes.size();
	const size_t radioconf = config_record_offset(bytes, ParamID::ARGOS_RADIOCONF);
	CHECK(radioconf != 0);
	std::vector<uint8_t> extra = uint_config_record(param_map[(unsigned int)ParamID::SHUTDOWN_NTIME_SAT].key, 5U);
	const std::vector<uint8_t> lbp14 =
	    uint_config_record(param_map[(unsigned int)ParamID::LB_SHUTDOWN_NTIME_SAT].key, 3U);
	extra.insert(extra.end(), lbp14.begin(), lbp14.end());
	bytes.insert(bytes.begin() + radioconf + CONFIG_RECORD, extra.begin(), extra.end());
	write_config_file(bytes);

	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	markers.check(*store);

	// Saved back in this build's layout, so the next boot reads it straight through.
	bytes = read_config_file();
	CHECK_EQUAL(this_build_size, bytes.size());
	CHECK_EQUAL((size_t)0, config_record_offset(bytes, ParamID::SHUTDOWN_NTIME_SAT));
}

// The file of a build without the camera, read by one with it: no record at
// all for slots 166-171. Those six come back at their defaults, nothing else.
TEST(ConfigStore, MissingRecordsResetOnlyTheirOwnSlots) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	const unsigned int cam_period_default = store->read_param<unsigned int>(ParamID::CAM_PERIOD_ON);
	const ConfigMarkers markers;
	markers.write(*store);
	store->write_param(ParamID::CAM_PERIOD_ON, cam_period_default + 7U);
	store->save_params();
	delete store;
	store = nullptr;

	std::vector<uint8_t> bytes = read_config_file();
	const size_t first = config_record_offset(bytes, ParamID::CAM_ENABLE);
	const size_t last = config_record_offset(bytes, ParamID::LB_CAM_EN);
	CHECK(first != 0);
	CHECK_EQUAL((size_t)5, (last - first) / CONFIG_RECORD);
	bytes.erase(bytes.begin() + first, bytes.begin() + last + CONFIG_RECORD);
	write_config_file(bytes);

	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	markers.check(*store);
	CHECK_EQUAL(cam_period_default, store->read_param<unsigned int>(ParamID::CAM_PERIOD_ON));
}

// A slot a later firmware added in the middle of the table, read after a
// downgrade: its key is nowhere in this build's table.
TEST(ConfigStore, ARecordWithAKeyThisBuildDoesNotKnowIsSkipped) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	const ConfigMarkers markers;
	markers.write(*store);
	store->save_params();
	delete store;
	store = nullptr;

	std::vector<uint8_t> bytes = read_config_file();
	const std::vector<uint8_t> unknown = uint_config_record("ZZP99", 42U);
	bytes.insert(bytes.begin() + CONFIG_HEADER + CONFIG_RECORD, unknown.begin(), unknown.end());
	write_config_file(bytes);

	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	markers.check(*store);
}

// A slot moved in the table: two records out of order are both placed.
TEST(ConfigStore, RecordsOutOfOrderAreAllPlaced) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	const ConfigMarkers markers;
	markers.write(*store);
	store->save_params();
	delete store;
	store = nullptr;

	std::vector<uint8_t> bytes = read_config_file();
	const size_t radioconf = config_record_offset(bytes, ParamID::ARGOS_RADIOCONF);
	const size_t rtc = config_record_offset(bytes, ParamID::LAST_KNOWN_RTC);
	CHECK(radioconf != 0 && rtc > radioconf);
	std::swap_ranges(bytes.begin() + radioconf, bytes.begin() + radioconf + CONFIG_RECORD, bytes.begin() + rtc);
	write_config_file(bytes);

	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	markers.check(*store);
}

// The normal boot: the file is exactly what this build writes, and reading it
// must not save it back (~0.4 s of flash per boot on LinkIt, at every TPL wake
// on RSPB). A padding byte, which the reader ignores and a save would zero,
// tells whether init() rewrote the file.
TEST(ConfigStore, AFileInThisBuildsLayoutIsNotRewrittenAtBoot) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	const ConfigMarkers markers;
	markers.write(*store);
	store->save_params();
	delete store;
	store = nullptr;

	std::vector<uint8_t> bytes = read_config_file();
	const size_t deep_idle = config_record_offset(bytes, ParamID::GNSS_DEEP_IDLE_AFTER_OFF_S);
	CHECK(deep_idle != 0);
	const size_t padding = deep_idle + CONFIG_RECORD - 1;
	bytes[padding] = 0xA5;
	write_config_file(bytes);

	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	markers.check(*store);
	CHECK_EQUAL(0xA5U, (unsigned int)read_config_file()[padding]);
}

TEST(ConfigStore, CheckConfigStoreResetsBadVariantType) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	// Modify some parameter values
	std::string model = "GenTracker";
	store->write_param(ParamID::ARGOS_DECID, model);
	store->save_params();

	// Delete the object and recreate a new one
	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();  // This will read in the saved file

	// Check default value has been restored
	CHECK_EQUAL(0U, store->read_param<unsigned int>(ParamID::ARGOS_DECID));
}

TEST(ConfigStore, CheckPartiallyCorruptedConfigurationStoreRetainsProtectedParams) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	// Set protected parameter values
	unsigned int dec_id = 1234U;
	unsigned int hex_id = 0x1234567U;
	store->write_param(ParamID::ARGOS_DECID, dec_id);
	store->write_param(ParamID::ARGOS_HEXID, hex_id);

	// Save parameters
	store->save_params();

	{
		// Overwrite first 4 bytes (configuration version)
		LFSFile f(main_filesystem, "config.dat", LFS_O_WRONLY);
		uint32_t clobbered_version = 0;
		f.write(&clobbered_version, sizeof(uint32_t));
	}

	// Delete the object and recreate a new one
	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();  // This will read in the partially saved file

	// Check default value has been restored
	CHECK_EQUAL(dec_id, store->read_param<unsigned int>(ParamID::ARGOS_DECID));
	CHECK_EQUAL(hex_id, store->read_param<unsigned int>(ParamID::ARGOS_HEXID));
}


TEST(ConfigStore, CheckFullyCorruptedConfigurationStore) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	// Set protected parameter values
	unsigned int dec_id = 1234U;
	unsigned int hex_id = 0x1234567U;
	store->write_param(ParamID::ARGOS_DECID, dec_id);
	store->write_param(ParamID::ARGOS_HEXID, hex_id);

	// Save parameters
	store->save_params();

	{
		// Overwrite first 1024 bytes (configuration version)
		LFSFile f(main_filesystem, "config.dat", LFS_O_WRONLY);
		uint8_t clobber[1024] = {};
		f.write(clobber, sizeof(clobber));
	}

	// Delete the object and recreate a new one
	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();  // This will read in the partially saved file

	// Check default values are restored
	CHECK_EQUAL(0U, store->read_param<unsigned int>(ParamID::ARGOS_DECID));
	CHECK_EQUAL(0U, store->read_param<unsigned int>(ParamID::ARGOS_HEXID));
}

TEST(ConfigStore, CheckFactoryResetRetainsProtectedParams) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	// Set protected parameter values
	unsigned int dec_id = 1234U;
	unsigned int hex_id = 0x1234567U;
	store->write_param(ParamID::ARGOS_DECID, dec_id);
	store->write_param(ParamID::ARGOS_HEXID, hex_id);

	// Save parameters
	store->save_params();

	// Factory reset
	store->factory_reset();

	// Delete the object and recreate a new one
	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();  // This will read in the partially saved file

	// Check default value has been restored
	CHECK_EQUAL(dec_id, store->read_param<unsigned int>(ParamID::ARGOS_DECID));
	CHECK_EQUAL(hex_id, store->read_param<unsigned int>(ParamID::ARGOS_HEXID));
}

// factory_reset is reached automatically after repeated failed boots. The
// provisioning secrets it promises to keep (PROTECTED_PARAMS) must come back on
// the next boot, or the recovered tag can no longer transmit. Only DECID/HEXID
// were checked above, and they are the two that sit at the head of the file.
TEST(ConfigStore, CheckFactoryResetRetainsProvisioningSecrets) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	const std::string radioconf = "0123456789ABCDEF0123456789ABCDEF";
	store->write_param(ParamID::ARGOS_RADIOCONF, radioconf);
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	const std::string appeui = "70B3D57ED0000001";
	const std::string appkey = "00112233445566778899AABBCCDDEEFF";
	store->write_param(ParamID::LORA_APPEUI, appeui);
	store->write_param(ParamID::LORA_APPKEY, appkey);
	// Join mode and band belong to provisioning too: an ABP tag, or one on
	// another band, keeps its keys but can no longer reach its network.
	const unsigned int njm = 0U;   // ABP, not the default OTAA
	const unsigned int band = 5U;  // not the default EU868
	store->write_param(ParamID::LORA_NJM, njm);
	store->write_param(ParamID::LORA_BAND, band);
#endif
	unsigned int tr_nom = 1200U;
	store->write_param(ParamID::TR_NOM, tr_nom);
	store->save_params();

	store->factory_reset();

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	STRCMP_EQUAL(radioconf.c_str(), store->read_param<std::string>(ParamID::ARGOS_RADIOCONF).c_str());
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	STRCMP_EQUAL(appeui.c_str(), store->read_param<std::string>(ParamID::LORA_APPEUI).c_str());
	STRCMP_EQUAL(appkey.c_str(), store->read_param<std::string>(ParamID::LORA_APPKEY).c_str());
	CHECK_EQUAL(njm, store->read_param<unsigned int>(ParamID::LORA_NJM));
	CHECK_EQUAL(band, store->read_param<unsigned int>(ParamID::LORA_BAND));
#endif
	// Everything that is not protected is back to its default.
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::TR_NOM) != tr_nom);
}

class DummyCalibration : public Calibratable {
public:
	DummyCalibration(const char *name) : Calibratable(name), m_cal(Calibration(name)) {}
	void calibration_write(const double value, const unsigned int offset) override { m_cal.write(offset, value); }
	void calibration_read(double &value, const unsigned int offset) override { value = m_cal.read(offset); }
	void calibration_save(bool force) override { m_cal.save(force); }

private:
	Calibration m_cal;
};

TEST(ConfigStore, CheckFactoryResetRetainsCalibrationData) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	{
		// Add calibration objects
		DummyCalibration cal("CAL");

		// Add data to calibration
		cal.calibration_write(1.0, 1);
		cal.calibration_write(2.0, 2);
		cal.calibration_write(3.0, 3);

		// Factory reset
		store->factory_reset();
	}

	// Delete the object and recreate a new one
	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();  // This will read in the partially saved file

	{
		// Create calibration object
		DummyCalibration cal("CAL");
		double value;

		// Check calibration points were retained
		cal.calibration_read(value, 1);
		CHECK_EQUAL(1.0, value);
		cal.calibration_read(value, 2);
		CHECK_EQUAL(2.0, value);
		cal.calibration_read(value, 3);
		CHECK_EQUAL(3.0, value);
	}
}

TEST(ConfigStore, CheckDefaultZoneSettings) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	CHECK_TRUE(store->read_param<BaseZoneType>(ParamID::ZONE_TYPE) == BaseZoneType::CIRCLE);
	CHECK_FALSE(store->read_param<bool>(ParamID::ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE));
	CHECK_TRUE(store->read_param<bool>(ParamID::ZONE_ENABLE_ACTIVATION_DATE));
	CHECK_TRUE(store->read_param<std::time_t>(ParamID::ZONE_ACTIVATION_DATE) == (std::time_t)1577836800U);
	CHECK_TRUE(store->read_param<BaseDepthPile>(ParamID::ZONE_ARGOS_DEPTH_PILE) == BaseDepthPile::DEPTH_PILE_1);
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::ZONE_ARGOS_REPETITION_SECONDS) == 240);
	CHECK_TRUE(store->read_param<BaseArgosMode>(ParamID::ZONE_ARGOS_MODE) == BaseArgosMode::LEGACY);
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::ZONE_ARGOS_DUTY_CYCLE) == 0xFFFFFFU);
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::ZONE_ARGOS_NTRY_PER_MESSAGE) == 0U);
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS) == 3600U);
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::ZONE_GNSS_HDOPFILT_THR) == 5U);
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::ZONE_GNSS_HACCFILT_THR) == 50U);
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::ZONE_GNSS_ACQ_TIMEOUT) == 240U);
	CHECK_TRUE(store->read_param<double>(ParamID::ZONE_CENTER_LONGITUDE) == -123.3925);
	CHECK_TRUE(store->read_param<double>(ParamID::ZONE_CENTER_LATITUDE) == -48.8752);
	CHECK_TRUE(store->read_param<unsigned int>(ParamID::ZONE_RADIUS) == 1000U);
}

TEST(ConfigStore, CheckDefaultPassPredictIsAvailable) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	BasePassPredict pp = store->read_pass_predict();
	// 25 satellites: the Kineis constellation replaced the 8-satellite Argos
	// table with the PREPASS v4.0 migration. Must track num_records in
	// ConfigurationStore::default_prepass (config_store.hpp), which is
	// protected and so cannot be referenced from here.
	CHECK_EQUAL(25, pp.num_records);
}

TEST(ConfigStore, CheckPassPredictCreationAndPersistence) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	{
		BasePassPredict pp;
		pp.num_records = 10;
		store->write_pass_predict(pp);
	}

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);

	{
		store->init();
		BasePassPredict &pp = store->read_pass_predict();
		CHECK_EQUAL(10, pp.num_records);
	}
}

TEST(ConfigStore, CheckPassPredictVersionCodeMismatch) {
	store = new LFSConfigurationStore(*main_filesystem);

	store->init();

	{
		BasePassPredict pp;
		pp.num_records = 10;
		store->write_pass_predict(pp);
	}

	delete store;

	// Corrupt the prepass file first 4 bytes
	{
		// Overwrite first 4 bytes (configuration version)
		LFSFile f(main_filesystem, "pass_predict.dat", LFS_O_WRONLY);
		uint8_t clobber[4] = {};
		f.write(clobber, sizeof(clobber));
	}

	store = new LFSConfigurationStore(*main_filesystem);

	{
		store->init();
		BasePassPredict pp = store->read_pass_predict();
		// 25 satellites: the Kineis constellation replaced the 8-satellite Argos
		// table with the PREPASS v4.0 migration. Must track num_records in
		// ConfigurationStore::default_prepass (config_store.hpp), which is
		// protected and so cannot be referenced from here.
		CHECK_EQUAL(25, pp.num_records);
	}
}

TEST(ConfigStore, PARAM_ARGOS_DECID) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int dec_id = 1234U;
	store->write_param(ParamID::ARGOS_DECID, dec_id);
	CHECK_EQUAL(dec_id, store->read_param<unsigned int>(ParamID::ARGOS_DECID));
}

TEST(ConfigStore, PARAM_ARGOS_HEXID) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int hex_id = 1234U;
	store->write_param(ParamID::ARGOS_HEXID, hex_id);
	CHECK_EQUAL(hex_id, store->read_param<unsigned int>(ParamID::ARGOS_HEXID));
}

TEST(ConfigStore, PARAM_FW_APP_VERSION) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	std::string s = "New Firmware App Version";
	store->write_param(ParamID::FW_APP_VERSION, s);
	CHECK_EQUAL("V0.1"s, store->read_param<std::string>(ParamID::FW_APP_VERSION));  // Should not change
}

TEST(ConfigStore, PARAM_LAST_TX) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	std::time_t t = 1234U;
	store->write_param(ParamID::LAST_TX, t);
	CHECK_EQUAL(t, store->read_param<std::time_t>(ParamID::LAST_TX));
}

TEST(ConfigStore, PARAM_TX_COUNTER) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 64U;
	store->write_param(ParamID::TX_COUNTER, t);
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::TX_COUNTER));
}


// Removed: PARAM_LAST_FULL_CHARGE_DATE — slot 7 freed (was unused dead param,
// no reliable hardware "charge complete" signal on this platform).
// _RESERVED_7 is now invisible to DTE (is_implemented=false).

TEST(ConfigStore, PARAM_PROFILE_NAME) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	std::string s = "New Profile Name";
	store->write_param(ParamID::PROFILE_NAME, s);
	CHECK_EQUAL(s, store->read_param<std::string>(ParamID::PROFILE_NAME));
}

TEST(ConfigStore, PARAM_ARGOS_AOP_DATE) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	std::time_t t = 123224U;
	store->write_param(ParamID::ARGOS_AOP_DATE, t);
	CHECK_EQUAL(t, store->read_param<std::time_t>(ParamID::ARGOS_AOP_DATE));
}

// PARAM_ARGOS_FREQ and PARAM_ARGOS_POWER tests removed: these params are obsolete (RADIOCONF controls power/frequency)

TEST(ConfigStore, PARAM_TR_NOM) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 45;
	store->write_param(ParamID::TR_NOM, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::TR_NOM));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::TR_NOM));
}

TEST(ConfigStore, PARAM_ARGOS_MODE) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	BaseArgosMode t = BaseArgosMode::DUTY_CYCLE;
	store->write_param(ParamID::ARGOS_MODE, t);
	store->save_params();
	CHECK_TRUE(t == store->read_param<BaseArgosMode>(ParamID::ARGOS_MODE));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_TRUE(t == store->read_param<BaseArgosMode>(ParamID::ARGOS_MODE));
}

TEST(ConfigStore, PARAM_NTRY_PER_MESSAGE) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 3;
	store->write_param(ParamID::NTRY_PER_MESSAGE, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::NTRY_PER_MESSAGE));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::NTRY_PER_MESSAGE));
}

TEST(ConfigStore, PARAM_BLIND_MODE_DEFAULTS_AND_ROUNDTRIP) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Defaults: blind off, retx_nb=4, retx_period_s=60 (no behavior change)
	CHECK_FALSE(store->read_param<bool>(ParamID::ARGOS_BLIND_EN));
	CHECK_EQUAL(4U, store->read_param<unsigned int>(ParamID::ARGOS_BLIND_RETX_NB));
	CHECK_EQUAL(60U, store->read_param<unsigned int>(ParamID::ARGOS_BLIND_RETX_PERIOD_S));

	// Round-trip persists across reload
	store->write_param(ParamID::ARGOS_BLIND_EN, true);
	store->write_param(ParamID::ARGOS_BLIND_RETX_NB, 7U);
	store->write_param(ParamID::ARGOS_BLIND_RETX_PERIOD_S, 90U);
	store->save_params();
	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	CHECK_TRUE(store->read_param<bool>(ParamID::ARGOS_BLIND_EN));
	CHECK_EQUAL(7U, store->read_param<unsigned int>(ParamID::ARGOS_BLIND_RETX_NB));
	CHECK_EQUAL(90U, store->read_param<unsigned int>(ParamID::ARGOS_BLIND_RETX_PERIOD_S));
}

TEST(ConfigStore, PARAM_DUTY_CYCLE) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 0b101010101010101010101010;
	store->write_param(ParamID::DUTY_CYCLE, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::DUTY_CYCLE));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::DUTY_CYCLE));
}

TEST(ConfigStore, PARAM_GNSS_EN) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	bool t = true;
	store->write_param(ParamID::GNSS_EN, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<bool>(ParamID::GNSS_EN));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<bool>(ParamID::GNSS_EN));
}

TEST(ConfigStore, PARAM_DLOC_ARG_NOM) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 10U;
	store->write_param(ParamID::DLOC_ARG_NOM, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::DLOC_ARG_NOM));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::DLOC_ARG_NOM));
}

TEST(ConfigStore, PARAM_ARGOS_DEPTH_PILE) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	BaseDepthPile t = BaseDepthPile::DEPTH_PILE_12;
	store->write_param(ParamID::ARGOS_DEPTH_PILE, t);
	store->save_params();
	CHECK_TRUE(t == store->read_param<BaseDepthPile>(ParamID::ARGOS_DEPTH_PILE));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_TRUE(t == store->read_param<BaseDepthPile>(ParamID::ARGOS_DEPTH_PILE));
}

TEST(ConfigStore, PARAM_GNSS_HDOPFILT_EN) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	bool t = true;
	store->write_param(ParamID::GNSS_HDOPFILT_EN, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<bool>(ParamID::GNSS_HDOPFILT_EN));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<bool>(ParamID::GNSS_HDOPFILT_EN));
}

TEST(ConfigStore, PARAM_GNSS_HDOPFILT_THR) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 10U;
	store->write_param(ParamID::GNSS_HDOPFILT_THR, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::GNSS_HDOPFILT_THR));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::GNSS_HDOPFILT_THR));
}

TEST(ConfigStore, PARAM_GNSS_ACQ_TIMEOUT) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 30U;
	store->write_param(ParamID::GNSS_ACQ_TIMEOUT, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::GNSS_ACQ_TIMEOUT));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::GNSS_ACQ_TIMEOUT));
}

TEST(ConfigStore, PARAM_UNDERWATER_EN) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	bool t = true;
	store->write_param(ParamID::UNDERWATER_EN, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<bool>(ParamID::UNDERWATER_EN));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<bool>(ParamID::UNDERWATER_EN));
}

TEST(ConfigStore, PARAM_DRY_TIME_BEFORE_TX) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 10U;
	store->write_param(ParamID::DRY_TIME_BEFORE_TX, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX));
}


TEST(ConfigStore, PARAM_SAMPLING_UNDER_FREQ) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	double t = 1440.0;
	store->write_param(ParamID::SAMPLING_UNDER_FREQ, t);
	store->save_params();
	DOUBLES_EQUAL(t, store->read_param<double>(ParamID::SAMPLING_UNDER_FREQ), 1e-9);

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	DOUBLES_EQUAL(t, store->read_param<double>(ParamID::SAMPLING_UNDER_FREQ), 1e-9);
}

TEST(ConfigStore, PARAM_LB_EN) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	bool t = true;
	store->write_param(ParamID::LB_EN, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<bool>(ParamID::LB_EN));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<bool>(ParamID::LB_EN));
}

TEST(ConfigStore, PARAM_LB_THRESHOLD) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 5;
	store->write_param(ParamID::LB_THRESHOLD, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::LB_THRESHOLD));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::LB_THRESHOLD));
}

// PARAM_LB_ARGOS_POWER test removed: param is obsolete (RADIOCONF controls power)

TEST(ConfigStore, PARAM_TR_LB) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 1200U;
	store->write_param(ParamID::TR_LB, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::TR_LB));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::TR_LB));
}

TEST(ConfigStore, PARAM_LB_ARGOS_MODE) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	BaseArgosMode t = BaseArgosMode::LEGACY;
	store->write_param(ParamID::LB_ARGOS_MODE, t);
	store->save_params();
	CHECK_TRUE(t == store->read_param<BaseArgosMode>(ParamID::LB_ARGOS_MODE));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_TRUE(t == store->read_param<BaseArgosMode>(ParamID::LB_ARGOS_MODE));
}

TEST(ConfigStore, PARAM_LB_ARGOS_DUTY_CYCLE) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 0b101010101010101010101010;
	store->write_param(ParamID::LB_ARGOS_DUTY_CYCLE, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::LB_ARGOS_DUTY_CYCLE));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::LB_ARGOS_DUTY_CYCLE));
}

TEST(ConfigStore, PARAM_LB_GNSS_EN) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	bool t = true;
	store->write_param(ParamID::LB_GNSS_EN, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<bool>(ParamID::LB_GNSS_EN));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<bool>(ParamID::LB_GNSS_EN));
}

TEST(ConfigStore, PARAM_DLOC_ARG_LB) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 720U;
	store->write_param(ParamID::DLOC_ARG_LB, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::DLOC_ARG_LB));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::DLOC_ARG_LB));
}

TEST(ConfigStore, PARAM_LB_GNSS_HDOPFILT_THR) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 10U;
	store->write_param(ParamID::LB_GNSS_HDOPFILT_THR, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::LB_GNSS_HDOPFILT_THR));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::LB_GNSS_HDOPFILT_THR));
}

TEST(ConfigStore, PARAM_LB_GNSS_ACQ_TIMEOUT) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int t = 30U;
	store->write_param(ParamID::LB_GNSS_ACQ_TIMEOUT, t);
	store->save_params();
	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::LB_GNSS_ACQ_TIMEOUT));

	delete store;
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_EQUAL(t, store->read_param<unsigned int>(ParamID::LB_GNSS_ACQ_TIMEOUT));
}

TEST(ConfigStore, RetrieveGPSConfigDefaultMode) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Set default params and LB params
	unsigned int hdop_filter_threshold = 10;
	bool hdop_filter_enable = true;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 1440 * 60U;
	unsigned int acquisition_timeout = 10;
	bool lb_en = false;
	unsigned int lb_hdop_filter_threshold = 5;
	bool lb_gnss_en = false;
	unsigned int lb_dloc_arg_nom = 720 * 60U;
	unsigned int lb_acquisition_timeout = 30;

	store->write_param(ParamID::GNSS_HDOPFILT_THR, hdop_filter_threshold);
	store->write_param(ParamID::GNSS_HDOPFILT_EN, hdop_filter_enable);
	store->write_param(ParamID::GNSS_EN, gnss_en);
	store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	store->write_param(ParamID::GNSS_ACQ_TIMEOUT, acquisition_timeout);
	store->write_param(ParamID::LB_EN, lb_en);
	store->write_param(ParamID::LB_GNSS_HDOPFILT_THR, lb_hdop_filter_threshold);
	store->write_param(ParamID::LB_GNSS_EN, lb_gnss_en);
	store->write_param(ParamID::DLOC_ARG_LB, lb_dloc_arg_nom);
	store->write_param(ParamID::LB_GNSS_ACQ_TIMEOUT, lb_acquisition_timeout);

	GNSSConfig gnss_config;
	store->get_gnss_configuration(gnss_config);

	CHECK_EQUAL(acquisition_timeout, gnss_config.acquisition_timeout);
	CHECK_TRUE(dloc_arg_nom == gnss_config.dloc_arg_nom);
	CHECK_EQUAL(gnss_en, gnss_config.enable);
	CHECK_EQUAL(hdop_filter_enable, gnss_config.hdop_filter_enable);
	CHECK_EQUAL(hdop_filter_threshold, gnss_config.hdop_filter_threshold);
}

TEST(ConfigStore, RetrieveGPSConfigLBMode) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Set default params and LB params
	unsigned int hdop_filter_threshold = 10;
	bool hdop_filter_enable = true;
	unsigned int hacc_filter_threshold = 50;
	bool hacc_filter_enable = true;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 1440 * 60U;
	unsigned int acquisition_timeout = 10;
	bool lb_en = true;
	unsigned int lb_hdop_filter_threshold = 5;
	unsigned int lb_hacc_filter_threshold = 100;
	bool lb_gnss_en = false;
	unsigned int lb_dloc_arg_nom = 720 * 60U;
	unsigned int lb_acquisition_timeout = 30;
	unsigned int lb_thresh = 10;

	store->write_param(ParamID::GNSS_HDOPFILT_THR, hdop_filter_threshold);
	store->write_param(ParamID::GNSS_HDOPFILT_EN, hdop_filter_enable);
	store->write_param(ParamID::GNSS_HACCFILT_THR, hacc_filter_threshold);
	store->write_param(ParamID::GNSS_HACCFILT_EN, hacc_filter_enable);
	store->write_param(ParamID::GNSS_EN, gnss_en);
	store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	store->write_param(ParamID::GNSS_ACQ_TIMEOUT, acquisition_timeout);
	store->write_param(ParamID::LB_EN, lb_en);
	store->write_param(ParamID::LB_GNSS_HDOPFILT_THR, lb_hdop_filter_threshold);
	store->write_param(ParamID::LB_GNSS_HACCFILT_THR, lb_hacc_filter_threshold);
	store->write_param(ParamID::LB_GNSS_EN, lb_gnss_en);
	store->write_param(ParamID::DLOC_ARG_LB, lb_dloc_arg_nom);
	store->write_param(ParamID::LB_GNSS_ACQ_TIMEOUT, lb_acquisition_timeout);
	store->write_param(ParamID::LB_THRESHOLD, lb_thresh);

	// Notify battery level above threshold
	fake_battery_monitor->set_values(100);

	GNSSConfig gnss_config;
	store->get_gnss_configuration(gnss_config);

	CHECK_EQUAL(acquisition_timeout, gnss_config.acquisition_timeout);
	CHECK_TRUE(dloc_arg_nom == gnss_config.dloc_arg_nom);
	CHECK_EQUAL(gnss_en, gnss_config.enable);
	CHECK_EQUAL(hdop_filter_enable, gnss_config.hdop_filter_enable);
	CHECK_EQUAL(hdop_filter_threshold, gnss_config.hdop_filter_threshold);
	CHECK_EQUAL(hacc_filter_enable, gnss_config.hacc_filter_enable);
	CHECK_EQUAL(hacc_filter_threshold, gnss_config.hacc_filter_threshold);

	// Notify battery level equal threshold
	fake_battery_monitor->set_values(10, 4200, true, false);

	store->get_gnss_configuration(gnss_config);

	CHECK_EQUAL(lb_acquisition_timeout, gnss_config.acquisition_timeout);
	CHECK_TRUE(lb_dloc_arg_nom == gnss_config.dloc_arg_nom);
	CHECK_EQUAL(lb_gnss_en, gnss_config.enable);
	CHECK_EQUAL(hdop_filter_enable, gnss_config.hdop_filter_enable);
	CHECK_EQUAL(lb_hdop_filter_threshold, gnss_config.hdop_filter_threshold);
	CHECK_EQUAL(hacc_filter_enable, gnss_config.hacc_filter_enable);
	CHECK_EQUAL(lb_hacc_filter_threshold, gnss_config.hacc_filter_threshold);

	// Notify battery level below threshold
	fake_battery_monitor->set_values(1, 4200, true, false);

	store->get_gnss_configuration(gnss_config);

	CHECK_EQUAL(lb_acquisition_timeout, gnss_config.acquisition_timeout);
	CHECK_TRUE(lb_dloc_arg_nom == gnss_config.dloc_arg_nom);
	CHECK_EQUAL(lb_gnss_en, gnss_config.enable);
	CHECK_EQUAL(hdop_filter_enable, gnss_config.hdop_filter_enable);
	CHECK_EQUAL(lb_hdop_filter_threshold, gnss_config.hdop_filter_threshold);
	CHECK_EQUAL(hacc_filter_enable, gnss_config.hacc_filter_enable);
	CHECK_EQUAL(lb_hacc_filter_threshold, gnss_config.hacc_filter_threshold);

	// Notify battery level 5% above threshold whilst in LB mode
	fake_battery_monitor->set_values(15, 4200, false, false);

	store->get_gnss_configuration(gnss_config);

	CHECK_EQUAL(acquisition_timeout, gnss_config.acquisition_timeout);
	CHECK_TRUE(dloc_arg_nom == gnss_config.dloc_arg_nom);
	CHECK_EQUAL(gnss_en, gnss_config.enable);
	CHECK_EQUAL(hdop_filter_enable, gnss_config.hdop_filter_enable);
	CHECK_EQUAL(hdop_filter_threshold, gnss_config.hdop_filter_threshold);
	CHECK_EQUAL(hacc_filter_enable, gnss_config.hacc_filter_enable);
	CHECK_EQUAL(hacc_filter_threshold, gnss_config.hacc_filter_threshold);
}

TEST(ConfigStore, RetrieveArgosConfigDefaultMode) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Set default params and LB params
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_12;
	unsigned int dry_time_before_tx = 10;
	unsigned int duty_cycle = 0xFFFFFFU;
	BaseArgosMode mode = BaseArgosMode::DUTY_CYCLE;
	unsigned int ntry_per_message = 1;
	unsigned int tr_nom = 60;
	unsigned int tx_counter = 12;
	bool lb_en = false;
	BaseDepthPile lb_depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int lb_duty_cycle = 0xAAAAAAU;
	BaseArgosMode lb_mode = BaseArgosMode::LEGACY;
	unsigned int lb_tr_nom = 120;

	store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	store->write_param(ParamID::DRY_TIME_BEFORE_TX, dry_time_before_tx);
	store->write_param(ParamID::DUTY_CYCLE, duty_cycle);
	store->write_param(ParamID::ARGOS_MODE, mode);
	store->write_param(ParamID::NTRY_PER_MESSAGE, ntry_per_message);
	store->write_param(ParamID::TR_NOM, tr_nom);
	store->write_param(ParamID::TX_COUNTER, tx_counter);
	store->write_param(ParamID::LB_EN, lb_en);
	store->write_param(ParamID::LB_ARGOS_DEPTH_PILE, lb_depth_pile);
	store->write_param(ParamID::LB_ARGOS_DUTY_CYCLE, lb_duty_cycle);
	store->write_param(ParamID::LB_ARGOS_MODE, lb_mode);
	store->write_param(ParamID::TR_LB, lb_tr_nom);

	ArgosConfig argos_config;
	store->get_argos_configuration(argos_config);

	CHECK_EQUAL((unsigned int)depth_pile, (unsigned int)argos_config.depth_pile);
	CHECK_EQUAL(dry_time_before_tx, argos_config.dry_time_before_tx);
	CHECK_EQUAL(duty_cycle, argos_config.duty_cycle);
	CHECK_EQUAL(401.65, argos_config.frequency);
	CHECK_EQUAL((unsigned int)mode, (unsigned int)argos_config.mode);
	CHECK_EQUAL(ntry_per_message, argos_config.ntry_per_message);
	CHECK_EQUAL((unsigned int)BaseArgosPower::POWER_350_MW, (unsigned int)argos_config.power);
	CHECK_EQUAL(tr_nom, argos_config.tx_interval_s);
	CHECK_EQUAL(tx_counter, argos_config.tx_counter);
}

TEST(ConfigStore, RetrieveArgosConfigLBMode) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Set default params and LB params
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_12;
	unsigned int dry_time_before_tx = 10;
	unsigned int duty_cycle = 0xFFFFFFU;
	BaseArgosMode mode = BaseArgosMode::DUTY_CYCLE;
	unsigned int ntry_per_message = 1;
	unsigned int tr_nom = 60;
	unsigned int tx_counter = 12;
	bool lb_en = true;
	BaseDepthPile lb_depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int lb_duty_cycle = 0xAAAAAAU;
	unsigned int lb_ntry_per_message = 5U;
	BaseArgosMode lb_mode = BaseArgosMode::LEGACY;
	unsigned int lb_tr_nom = 120;
	unsigned int lb_thresh = 10U;

	store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	store->write_param(ParamID::DRY_TIME_BEFORE_TX, dry_time_before_tx);
	store->write_param(ParamID::DUTY_CYCLE, duty_cycle);
	store->write_param(ParamID::ARGOS_MODE, mode);
	store->write_param(ParamID::NTRY_PER_MESSAGE, ntry_per_message);
	store->write_param(ParamID::TR_NOM, tr_nom);
	store->write_param(ParamID::TX_COUNTER, tx_counter);
	store->write_param(ParamID::LB_EN, lb_en);
	store->write_param(ParamID::LB_NTRY_PER_MESSAGE, lb_ntry_per_message);
	store->write_param(ParamID::LB_ARGOS_DEPTH_PILE, lb_depth_pile);
	store->write_param(ParamID::LB_ARGOS_DUTY_CYCLE, lb_duty_cycle);
	store->write_param(ParamID::LB_ARGOS_MODE, lb_mode);
	store->write_param(ParamID::TR_LB, lb_tr_nom);
	store->write_param(ParamID::LB_THRESHOLD, lb_thresh);

	// Notify battery level above threshold
	fake_battery_monitor->set_values(100);

	ArgosConfig argos_config;
	store->get_argos_configuration(argos_config);

	CHECK_EQUAL((unsigned int)depth_pile, (unsigned int)argos_config.depth_pile);
	CHECK_EQUAL(dry_time_before_tx, argos_config.dry_time_before_tx);
	CHECK_EQUAL(duty_cycle, argos_config.duty_cycle);
	CHECK_EQUAL(401.65, argos_config.frequency);
	CHECK_EQUAL((unsigned int)mode, (unsigned int)argos_config.mode);
	CHECK_EQUAL(ntry_per_message, argos_config.ntry_per_message);
	CHECK_EQUAL((unsigned int)BaseArgosPower::POWER_350_MW, (unsigned int)argos_config.power);
	CHECK_EQUAL(tr_nom, argos_config.tx_interval_s);
	CHECK_EQUAL(tx_counter, argos_config.tx_counter);

	// Notify battery level equal threshold
	fake_battery_monitor->set_values(10, 4200, true, false);

	store->get_argos_configuration(argos_config);

	CHECK_EQUAL((unsigned int)lb_depth_pile, (unsigned int)argos_config.depth_pile);
	CHECK_EQUAL(dry_time_before_tx, argos_config.dry_time_before_tx);
	CHECK_EQUAL(lb_duty_cycle, argos_config.duty_cycle);
	CHECK_EQUAL(401.65, argos_config.frequency);
	CHECK_EQUAL((unsigned int)lb_mode, (unsigned int)argos_config.mode);
	CHECK_EQUAL(lb_ntry_per_message, argos_config.ntry_per_message);
	CHECK_EQUAL((unsigned int)BaseArgosPower::POWER_350_MW, (unsigned int)argos_config.power);
	CHECK_EQUAL(lb_tr_nom, argos_config.tx_interval_s);
	CHECK_EQUAL(tx_counter, argos_config.tx_counter);

	// Notify battery level below threshold
	fake_battery_monitor->set_values(1, 4200, true, false);

	store->get_argos_configuration(argos_config);

	CHECK_EQUAL((unsigned int)lb_depth_pile, (unsigned int)argos_config.depth_pile);
	CHECK_EQUAL(dry_time_before_tx, argos_config.dry_time_before_tx);
	CHECK_EQUAL(lb_duty_cycle, argos_config.duty_cycle);
	CHECK_EQUAL(401.65, argos_config.frequency);
	CHECK_EQUAL((unsigned int)lb_mode, (unsigned int)argos_config.mode);
	CHECK_EQUAL(lb_ntry_per_message, argos_config.ntry_per_message);
	CHECK_EQUAL((unsigned int)BaseArgosPower::POWER_350_MW, (unsigned int)argos_config.power);
	CHECK_EQUAL(lb_tr_nom, argos_config.tx_interval_s);
	CHECK_EQUAL(tx_counter, argos_config.tx_counter);

	// Notify battery level 5% above threshold
	fake_battery_monitor->set_values(15, 4200, false, false);

	store->get_argos_configuration(argos_config);

	CHECK_EQUAL((unsigned int)depth_pile, (unsigned int)argos_config.depth_pile);
	CHECK_EQUAL(dry_time_before_tx, argos_config.dry_time_before_tx);
	CHECK_EQUAL(duty_cycle, argos_config.duty_cycle);
	CHECK_EQUAL(401.65, argos_config.frequency);
	CHECK_EQUAL((unsigned int)mode, (unsigned int)argos_config.mode);
	CHECK_EQUAL(ntry_per_message, argos_config.ntry_per_message);
	CHECK_EQUAL((unsigned int)BaseArgosPower::POWER_350_MW, (unsigned int)argos_config.power);
	CHECK_EQUAL(tr_nom, argos_config.tx_interval_s);
	CHECK_EQUAL(tx_counter, argos_config.tx_counter);
}

TEST(ConfigStore, ZoneExclusionCriteriaChecking) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_FALSE(store->is_zone_exclusion());

	// Set last GPS coordinates
	GPSLogEntry gps_location;
	gps_location.info.day = 1;
	gps_location.info.month = 1;
	gps_location.info.year = 2021;
	gps_location.info.hour = 0;
	gps_location.info.min = 0;
	gps_location.info.sec = 0;
	gps_location.info.valid = 1;
	gps_location.info.lon = -2.118413;
	gps_location.info.lat = 51.3765242;
	store->notify_gps_location(gps_location);

	// Set up exclusion zone
	BaseZoneType zone_type = BaseZoneType::CIRCLE;
	store->write_param(ParamID::ZONE_TYPE, zone_type);
	double zone_center_longitude_x = -2.118413;
	store->write_param(ParamID::ZONE_CENTER_LONGITUDE, zone_center_longitude_x);
	double zone_center_latitude_y = 51.3765242;
	store->write_param(ParamID::ZONE_CENTER_LATITUDE, zone_center_latitude_y);
	unsigned int zone_delta_arg_loc_argos_seconds = 100;
	store->write_param(ParamID::ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS, zone_delta_arg_loc_argos_seconds);
	unsigned int zone_radius_m = 100000;  // 100 km
	store->write_param(ParamID::ZONE_RADIUS, zone_radius_m);
	bool zone_enable_out_of_zone_detection_mode = true;
	store->write_param(ParamID::ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE, zone_enable_out_of_zone_detection_mode);
	unsigned int zone_gnss_acquisition_timeout_seconds = 30;
	store->write_param(ParamID::ZONE_GNSS_ACQ_TIMEOUT, zone_gnss_acquisition_timeout_seconds);
	unsigned int zone_hdop_filter_threshold = 3;
	store->write_param(ParamID::ZONE_GNSS_HDOPFILT_THR, zone_hdop_filter_threshold);

	// Inside zone
	CHECK_FALSE(store->is_zone_exclusion());

	// Outside zone
	zone_center_longitude_x = -1.0;
	store->write_param(ParamID::ZONE_CENTER_LONGITUDE, zone_center_longitude_x);
	zone_center_latitude_y = 53;
	store->write_param(ParamID::ZONE_CENTER_LATITUDE, zone_center_latitude_y);
	CHECK_TRUE(store->is_zone_exclusion());

	// Enable activation date later than GPS time
	bool zone_enable_activation_date = true;
	store->write_param(ParamID::ZONE_ENABLE_ACTIVATION_DATE, zone_enable_activation_date);
	std::time_t zone_activation_date = convert_epochtime(2021, 2, 1, 0, 0, 0);
	store->write_param(ParamID::ZONE_ACTIVATION_DATE, zone_activation_date);
	CHECK_FALSE(store->is_zone_exclusion());

	// Enable activation date before GPS time
	zone_activation_date = convert_epochtime(2020, 12, 31, 23, 59, 0);
	store->write_param(ParamID::ZONE_ACTIVATION_DATE, zone_activation_date);
	CHECK_TRUE(store->is_zone_exclusion());

	// Put back inside zone
	zone_center_longitude_x = -2.118413;
	store->write_param(ParamID::ZONE_CENTER_LONGITUDE, zone_center_longitude_x);
	zone_center_latitude_y = 51.3765242;
	store->write_param(ParamID::ZONE_CENTER_LATITUDE, zone_center_latitude_y);
	CHECK_FALSE(store->is_zone_exclusion());

	// Outside zone but monitoring disabled
	zone_enable_out_of_zone_detection_mode = false;
	store->write_param(ParamID::ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE, zone_enable_out_of_zone_detection_mode);
	zone_center_longitude_x = -1.0;
	store->write_param(ParamID::ZONE_CENTER_LONGITUDE, zone_center_longitude_x);
	zone_center_latitude_y = 53;
	store->write_param(ParamID::ZONE_CENTER_LATITUDE, zone_center_latitude_y);
	CHECK_FALSE(store->is_zone_exclusion());
}

TEST(ConfigStore, RetrieveArgosConfigZoneExclusionMode) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Set last GPS coordinates
	GPSLogEntry gps_location;
	gps_location.info.day = 1;
	gps_location.info.month = 1;
	gps_location.info.year = 2021;
	gps_location.info.hour = 0;
	gps_location.info.min = 0;
	gps_location.info.sec = 0;
	gps_location.info.valid = 1;
	gps_location.info.lon = -2.118413;
	gps_location.info.lat = 51.3765242;
	store->notify_gps_location(gps_location);

	// Set up exclusion zone
	BaseZoneType zone_type = BaseZoneType::CIRCLE;
	store->write_param(ParamID::ZONE_TYPE, zone_type);
	double zone_center_longitude_x = -2.118413;
	store->write_param(ParamID::ZONE_CENTER_LONGITUDE, zone_center_longitude_x);
	double zone_center_latitude_y = 51.3765242;
	store->write_param(ParamID::ZONE_CENTER_LATITUDE, zone_center_latitude_y);
	unsigned int zone_delta_arg_loc_argos_seconds = 100;
	store->write_param(ParamID::ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS, zone_delta_arg_loc_argos_seconds);
	unsigned int zone_radius_m = 100000;  // 100 km
	store->write_param(ParamID::ZONE_RADIUS, zone_radius_m);
	bool zone_enable_out_of_zone_detection_mode = true;
	store->write_param(ParamID::ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE, zone_enable_out_of_zone_detection_mode);
	unsigned int zone_gnss_acquisition_timeout_seconds = 30;
	store->write_param(ParamID::ZONE_GNSS_ACQ_TIMEOUT, zone_gnss_acquisition_timeout_seconds);
	unsigned int zone_hdop_filter_threshold = 3;
	store->write_param(ParamID::ZONE_GNSS_HDOPFILT_THR, zone_hdop_filter_threshold);
	unsigned int zone_argos_duty_cycle = 0xAAAAAAU;
	store->write_param(ParamID::ZONE_ARGOS_DUTY_CYCLE, zone_argos_duty_cycle);
	unsigned int zone_argos_ntry_per_message = 4;
	store->write_param(ParamID::ZONE_ARGOS_NTRY_PER_MESSAGE, zone_argos_ntry_per_message);
	BaseDepthPile zone_argos_depth_pile = BaseDepthPile::DEPTH_PILE_1;
	store->write_param(ParamID::ZONE_ARGOS_DEPTH_PILE, zone_argos_depth_pile);
	BaseArgosMode zone_argos_mode = BaseArgosMode::LEGACY;
	store->write_param(ParamID::ZONE_ARGOS_MODE, zone_argos_mode);
	unsigned int zone_argos_time_repetition_seconds = 120;
	store->write_param(ParamID::ZONE_ARGOS_REPETITION_SECONDS, zone_argos_time_repetition_seconds);

	// Set default params
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_12;
	unsigned int dry_time_before_tx = 10;
	unsigned int duty_cycle = 0xFFFFFFU;
	BaseArgosMode mode = BaseArgosMode::DUTY_CYCLE;
	unsigned int ntry_per_message = 1;
	unsigned int tr_nom = 60;
	unsigned int tx_counter = 12;
	bool lb_en = false;

	store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	store->write_param(ParamID::DRY_TIME_BEFORE_TX, dry_time_before_tx);
	store->write_param(ParamID::DUTY_CYCLE, duty_cycle);
	store->write_param(ParamID::ARGOS_MODE, mode);
	store->write_param(ParamID::NTRY_PER_MESSAGE, ntry_per_message);
	store->write_param(ParamID::TR_NOM, tr_nom);
	store->write_param(ParamID::TX_COUNTER, tx_counter);
	store->write_param(ParamID::LB_EN, lb_en);

	// Inside zone, default params should apply
	ArgosConfig argos_config;
	store->get_argos_configuration(argos_config);

	CHECK_EQUAL((unsigned int)depth_pile, (unsigned int)argos_config.depth_pile);
	CHECK_EQUAL(dry_time_before_tx, argos_config.dry_time_before_tx);
	CHECK_EQUAL(duty_cycle, argos_config.duty_cycle);
	CHECK_EQUAL(401.65, argos_config.frequency);
	CHECK_EQUAL((unsigned int)mode, (unsigned int)argos_config.mode);
	CHECK_EQUAL(ntry_per_message, argos_config.ntry_per_message);
	CHECK_EQUAL((unsigned int)BaseArgosPower::POWER_350_MW, (unsigned int)argos_config.power);
	CHECK_EQUAL(tr_nom, argos_config.tx_interval_s);
	CHECK_EQUAL(tx_counter, argos_config.tx_counter);

	// Set outside zone
	zone_center_longitude_x = -1.0;
	store->write_param(ParamID::ZONE_CENTER_LONGITUDE, zone_center_longitude_x);
	zone_center_latitude_y = 53;
	store->write_param(ParamID::ZONE_CENTER_LATITUDE, zone_center_longitude_x);

	store->get_argos_configuration(argos_config);

	CHECK_EQUAL((unsigned int)zone_argos_depth_pile, (unsigned int)argos_config.depth_pile);
	CHECK_EQUAL(dry_time_before_tx, argos_config.dry_time_before_tx);
	CHECK_EQUAL(zone_argos_duty_cycle, argos_config.duty_cycle);
	CHECK_EQUAL(401.65, argos_config.frequency);
	CHECK_EQUAL((unsigned int)zone_argos_mode, (unsigned int)argos_config.mode);
	CHECK_EQUAL(zone_argos_ntry_per_message, argos_config.ntry_per_message);
	CHECK_EQUAL((unsigned int)BaseArgosPower::POWER_350_MW, (unsigned int)argos_config.power);
	CHECK_EQUAL(zone_argos_time_repetition_seconds, argos_config.tx_interval_s);
	CHECK_EQUAL(tx_counter, argos_config.tx_counter);
}


TEST(ConfigStore, RetrieveGPSConfigZoneExclusionMode) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Set last GPS coordinates
	GPSLogEntry gps_location;
	gps_location.info.day = 1;
	gps_location.info.month = 1;
	gps_location.info.year = 2021;
	gps_location.info.hour = 0;
	gps_location.info.min = 0;
	gps_location.info.sec = 0;
	gps_location.info.valid = 1;
	gps_location.info.lon = -2.118413;
	gps_location.info.lat = 51.3765242;
	store->notify_gps_location(gps_location);

	// Set up exclusion zone
	BaseZoneType zone_type = BaseZoneType::CIRCLE;
	store->write_param(ParamID::ZONE_TYPE, zone_type);
	double zone_center_longitude_x = -2.118413;
	store->write_param(ParamID::ZONE_CENTER_LONGITUDE, zone_center_longitude_x);
	double zone_center_latitude_y = 51.3765242;
	store->write_param(ParamID::ZONE_CENTER_LATITUDE, zone_center_latitude_y);
	unsigned int zone_delta_arg_loc_argos_seconds = 100;
	store->write_param(ParamID::ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS, zone_delta_arg_loc_argos_seconds);
	unsigned int zone_radius_m = 100000;  // 100 km
	store->write_param(ParamID::ZONE_RADIUS, zone_radius_m);
	bool zone_enable_out_of_zone_detection_mode = true;
	store->write_param(ParamID::ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE, zone_enable_out_of_zone_detection_mode);
	unsigned int zone_gnss_acquisition_timeout_seconds = 30;
	store->write_param(ParamID::ZONE_GNSS_ACQ_TIMEOUT, zone_gnss_acquisition_timeout_seconds);
	unsigned int zone_hdop_filter_threshold = 3;
	store->write_param(ParamID::ZONE_GNSS_HDOPFILT_THR, zone_hdop_filter_threshold);

	// Set default params
	unsigned int hdop_filter_threshold = 10;
	bool hdop_filter_enable = true;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 1440 * 60U;
	unsigned int acquisition_timeout = 10;
	bool lb_en = false;

	store->write_param(ParamID::GNSS_HDOPFILT_THR, hdop_filter_threshold);
	store->write_param(ParamID::GNSS_HDOPFILT_EN, hdop_filter_enable);
	store->write_param(ParamID::GNSS_EN, gnss_en);
	store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	store->write_param(ParamID::GNSS_ACQ_TIMEOUT, acquisition_timeout);
	store->write_param(ParamID::LB_EN, lb_en);

	// Inside zone, default params should apply

	GNSSConfig gnss_config;
	store->get_gnss_configuration(gnss_config);

	CHECK_EQUAL(acquisition_timeout, gnss_config.acquisition_timeout);
	CHECK_EQUAL((unsigned int)dloc_arg_nom, (unsigned int)gnss_config.dloc_arg_nom);
	CHECK_EQUAL(gnss_en, gnss_config.enable);
	CHECK_EQUAL(hdop_filter_enable, gnss_config.hdop_filter_enable);
	CHECK_EQUAL(hdop_filter_threshold, gnss_config.hdop_filter_threshold);

	// Set outside zone (DLOC specified)
	zone_center_longitude_x = -1.0;
	store->write_param(ParamID::ZONE_CENTER_LONGITUDE, zone_center_longitude_x);
	zone_center_latitude_y = 53;
	store->write_param(ParamID::ZONE_CENTER_LATITUDE, zone_center_latitude_y);

	store->get_gnss_configuration(gnss_config);

	CHECK_EQUAL(zone_gnss_acquisition_timeout_seconds, gnss_config.acquisition_timeout);
	CHECK_EQUAL((unsigned int)zone_delta_arg_loc_argos_seconds, (unsigned int)gnss_config.dloc_arg_nom);
	CHECK_EQUAL(gnss_en, gnss_config.enable);
	CHECK_EQUAL(hdop_filter_enable, gnss_config.hdop_filter_enable);
	CHECK_EQUAL(zone_hdop_filter_threshold, gnss_config.hdop_filter_threshold);
}


// ======== SPEC-EMB-002: Credentials dirty flag tests ========

TEST(ConfigStore, CredentialsDirtyOnInit) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Credentials should be dirty on first init (ensures first-boot write)
	CHECK_TRUE(store->is_credentials_dirty());
}

TEST(ConfigStore, CredentialsDirtyClearedAfterClear) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	CHECK_TRUE(store->is_credentials_dirty());
	store->clear_credentials_dirty();
	CHECK_FALSE(store->is_credentials_dirty());
}

TEST(ConfigStore, CredentialsDirtyResetOnArgosHexIdWrite) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	store->clear_credentials_dirty();
	CHECK_FALSE(store->is_credentials_dirty());

	// Writing ARGOS_HEXID should re-set the dirty flag
	unsigned int hex_id = 0x12345678;
	store->write_param(ParamID::ARGOS_HEXID, hex_id);
	CHECK_TRUE(store->is_credentials_dirty());
}

TEST(ConfigStore, CredentialsDirtyResetOnRadioconfWrite) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	store->clear_credentials_dirty();
	CHECK_FALSE(store->is_credentials_dirty());

	// Writing ARGOS_RADIOCONF should re-set the dirty flag
	std::string rconf = "550b4bec21009c7a7b5bebaa937cdb41"s;
	store->write_param(ParamID::ARGOS_RADIOCONF, rconf);
	CHECK_TRUE(store->is_credentials_dirty());
}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
TEST(ConfigStore, CredentialsDirtyResetOnSecKeyWrite) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	store->clear_credentials_dirty();
	CHECK_FALSE(store->is_credentials_dirty());

	// Writing ARGOS_SECKEY should re-set the dirty flag
	std::string seckey = "0123456789ABCDEF0123456789ABCDEF"s;
	store->write_param(ParamID::ARGOS_SECKEY, seckey);
	CHECK_TRUE(store->is_credentials_dirty());
}
#endif

TEST(ConfigStore, CredentialsDirtyNotAffectedByUnrelatedWrite) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	store->clear_credentials_dirty();
	CHECK_FALSE(store->is_credentials_dirty());

	// Writing an unrelated param should NOT set the dirty flag
	unsigned int tr_nom = 60U;
	store->write_param(ParamID::TR_NOM, tr_nom);
	CHECK_FALSE(store->is_credentials_dirty());
}

TEST(ConfigStore, CooldownDefaultIs2700) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	unsigned int interval = store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
	CHECK_EQUAL(2700U, interval);
}

// get_tx_jitter_seed() is the single seed source shared by ArgosTxService and
// LoRaTxService. The LoRa cases are the ones that bite in the field: a
// LoRa-only build never writes either Argos ID param, so without a DevEUI-based
// seed every unit would seed mt19937(0) and jitter identically.
TEST(ConfigStore, TxJitterSeedPrefersLoraDevEui) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	// Argos IDs present but meaningless on a LoRa build — DevEUI must win.
	store->write_param(ParamID::ARGOS_DECID, 1234U);
	store->write_param(ParamID::ARGOS_HEXID, 0x0BADF00DU);
	store->write_param(ParamID::LORA_DEVEUI, std::string("70B3D57ED0001234"));

	unsigned int seed = store->get_tx_jitter_seed();
	CHECK(seed != 0U);
	CHECK(seed != 1234U);
	CHECK(seed != 0x0BADF00DU);
}

// DevEUIs are normally allocated as a contiguous block behind a shared vendor
// prefix, so the fold has to stay sensitive to the LOW-order digits. Two units
// one apart must not collide.
TEST(ConfigStore, TxJitterSeedSeparatesAdjacentDevEuisInABatch) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	store->write_param(ParamID::LORA_DEVEUI, std::string("70B3D57ED0001234"));
	unsigned int a = store->get_tx_jitter_seed();

	store->write_param(ParamID::LORA_DEVEUI, std::string("70B3D57ED0001235"));
	unsigned int b = store->get_tx_jitter_seed();

	CHECK(a != b);
}

TEST(ConfigStore, TxJitterSeedPrefersArgosDecIdWhenNoDevEui) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	store->write_param(ParamID::LORA_DEVEUI, std::string(""));
	store->write_param(ParamID::ARGOS_DECID, 1234U);
	store->write_param(ParamID::ARGOS_HEXID, 0x0BADF00DU);

	CHECK_EQUAL(1234U, store->get_tx_jitter_seed());
}

TEST(ConfigStore, TxJitterSeedFallsBackToArgosHexId) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	store->write_param(ParamID::LORA_DEVEUI, std::string(""));
	store->write_param(ParamID::ARGOS_DECID, 0U);
	store->write_param(ParamID::ARGOS_HEXID, 0x0BADF00DU);

	CHECK_EQUAL(0x0BADF00DU, store->get_tx_jitter_seed());
}

TEST(ConfigStore, TxJitterSeedFallsBackToDeviceIdWhenNothingProvisioned) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();

	store->write_param(ParamID::LORA_DEVEUI, std::string(""));
	store->write_param(ParamID::ARGOS_DECID, 0U);
	store->write_param(ParamID::ARGOS_HEXID, 0U);

	// Must be unique-per-chip, never 0.
	CHECK_EQUAL((unsigned int)PMU::device_identifier(), store->get_tx_jitter_seed());
	CHECK(store->get_tx_jitter_seed() != 0U);
}

// === MOORED override in the configuration cascade (2026-08) ===============
//
// The classifier itself is covered by moored_mode_test.cpp; what matters here
// is that it plugs into ConfigurationStore without disturbing the existing
// LOW_BATTERY / HAULED / OUT_OF_ZONE / NORMAL precedence.

TEST(ConfigStore, MooredDisabledLeavesConfigurationUntouched) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	configuration_store = store;  // the classifiers read the global
	MooredModeService::reset_for_tests();
	HauledModeService::reset_for_tests();

	store->write_param(ParamID::DLOC_ARG_NOM, 60U);
	store->write_param(ParamID::TR_NOM, 60U);
	store->write_param(ParamID::GNSS_EN, (bool)true);

	GNSSConfig gnss;
	ArgosConfig argos;
	store->get_gnss_configuration(gnss);
	store->get_argos_configuration(argos);
	unsigned int base_dloc = gnss.dloc_arg_nom;
	unsigned int base_tr = argos.tx_interval_s;

	// Feed enough evidence that the classifier WOULD engage were it enabled.
	for (unsigned int i = 0; i < 10; i++)
		MooredModeService::on_gnss_fix(34.75, 33.03, 0, 1000 + i);
	CHECK_FALSE(MooredModeService::is_moored());

	store->get_gnss_configuration(gnss);
	store->get_argos_configuration(argos);
	CHECK_EQUAL(base_dloc, gnss.dloc_arg_nom);
	CHECK_EQUAL(base_tr, argos.tx_interval_s);
	CHECK_TRUE(gnss.enable);
}

TEST(ConfigStore, MooredSubstitutesGnssPeriodAndTxInterval) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	configuration_store = store;  // the classifiers read the global
	MooredModeService::reset_for_tests();
	HauledModeService::reset_for_tests();

	store->write_param(ParamID::DLOC_ARG_NOM, 60U);
	store->write_param(ParamID::TR_NOM, 60U);
	store->write_param(ParamID::GNSS_EN, (bool)true);
	store->write_param(ParamID::MOORED_DETECT_EN, (bool)true);
	store->write_param(ParamID::MOORED_ENTER_FIXES, 2U);
	store->write_param(ParamID::MOORED_DLOC, 7200U);     // 2 h
	store->write_param(ParamID::MOORED_TR_NOM, 10800U);  // 3 h
	store->write_param(ParamID::MOORED_GNSS_EN, (bool)true);

	for (unsigned int i = 0; i < 3; i++)
		MooredModeService::on_gnss_fix(34.75, 33.03, 0, 1000 + i);
	CHECK_TRUE(MooredModeService::is_moored());

	GNSSConfig gnss;
	ArgosConfig argos;
	store->get_gnss_configuration(gnss);
	store->get_argos_configuration(argos);
	CHECK_EQUAL(7200U, gnss.dloc_arg_nom);
	CHECK_EQUAL(10800U, argos.tx_interval_s);
	CHECK_TRUE(gnss.enable);

	// Under way again -> straight back to the base cadence.
	MooredModeService::on_gnss_fix(34.76, 33.03, 0, 2000);
	store->get_gnss_configuration(gnss);
	store->get_argos_configuration(argos);
	CHECK_EQUAL(60U, gnss.dloc_arg_nom);
	CHECK_EQUAL(60U, argos.tx_interval_s);
}

TEST(ConfigStore, MooredGnssEnCanOnlyNarrowNeverWiden) {
	// A deployment running GNSS_EN=0 must not be switched back on by the
	// moored branch, whatever MOORED_GNSS_EN says.
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	configuration_store = store;  // the classifiers read the global
	MooredModeService::reset_for_tests();
	HauledModeService::reset_for_tests();

	store->write_param(ParamID::GNSS_EN, (bool)false);
	store->write_param(ParamID::MOORED_DETECT_EN, (bool)true);
	store->write_param(ParamID::MOORED_ENTER_FIXES, 2U);
	store->write_param(ParamID::MOORED_GNSS_EN, (bool)true);

	for (unsigned int i = 0; i < 3; i++)
		MooredModeService::on_gnss_fix(34.75, 33.03, 0, 1000 + i);
	CHECK_TRUE(MooredModeService::is_moored());

	GNSSConfig gnss;
	store->get_gnss_configuration(gnss);
	CHECK_FALSE(gnss.enable);

	// And the converse: MOORED_GNSS_EN=0 turns GNSS off while moored.
	store->write_param(ParamID::GNSS_EN, (bool)true);
	store->write_param(ParamID::MOORED_GNSS_EN, (bool)false);
	store->get_gnss_configuration(gnss);
	CHECK_FALSE(gnss.enable);
}

TEST(ConfigStore, HauledTakesPriorityOverMoored) {
	// Priority cascade is LOW_BATTERY > HAULED > MOORED > OUT_OF_ZONE > NORMAL.
	// A hauled-out animal that is also stationary must keep the HAULED values.
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	configuration_store = store;  // the classifiers read the global
	MooredModeService::reset_for_tests();
	HauledModeService::reset_for_tests();

	store->write_param(ParamID::TR_NOM, 60U);
	store->write_param(ParamID::MOORED_DETECT_EN, (bool)true);
	store->write_param(ParamID::MOORED_ENTER_FIXES, 2U);
	store->write_param(ParamID::MOORED_TR_NOM, 10800U);
	store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	store->write_param(ParamID::HAULED_TR_NOM, 1800U);
	store->write_param(ParamID::HAULED_ARGOS_MODE, BaseArgosMode::LEGACY);

	for (unsigned int i = 0; i < 3; i++)
		MooredModeService::on_gnss_fix(34.75, 33.03, 0, 1000 + i);
	CHECK_TRUE(MooredModeService::is_moored());

	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 2 * 3600);
	CHECK_TRUE(HauledModeService::is_hauled());

	ArgosConfig argos;
	store->get_argos_configuration(argos);
	CHECK_EQUAL(1800U, argos.tx_interval_s);  // HAULED wins
}

TEST(ConfigStore, LowBatteryTakesPriorityOverMoored) {
	store = new LFSConfigurationStore(*main_filesystem);
	store->init();
	configuration_store = store;  // the classifiers read the global
	MooredModeService::reset_for_tests();
	HauledModeService::reset_for_tests();

	store->write_param(ParamID::TR_LB, 900U);
	store->write_param(ParamID::LB_EN, (bool)true);
	store->write_param(ParamID::MOORED_DETECT_EN, (bool)true);
	store->write_param(ParamID::MOORED_ENTER_FIXES, 2U);
	store->write_param(ParamID::MOORED_TR_NOM, 10800U);

	for (unsigned int i = 0; i < 3; i++)
		MooredModeService::on_gnss_fix(34.75, 33.03, 0, 1000 + i);
	CHECK_TRUE(MooredModeService::is_moored());

	fake_battery_monitor->set_values(5, 3300, /*is_low*/ true, /*is_critical*/ false);

	ArgosConfig argos;
	store->get_argos_configuration(argos);
	CHECK_TRUE(argos.is_lb);
	CHECK_EQUAL(900U, argos.tx_interval_s);  // LOW_BATTERY wins
}
