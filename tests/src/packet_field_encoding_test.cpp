#include "packet_field_encoding.hpp"

#include "argos_packet_builder.hpp"
#include "lora_packet_builder.hpp"

#include "CppUTest/TestHarness.h"

// These six encodings are a contract with a ground-segment decoder that lives
// outside this repository. Both packet builders must produce exactly the same
// bits for the same input -- which is why the code used to be duplicated
// byte-for-byte in each of them, with a comment saying so.
//
// Now that there is one implementation, these tests are what stops it drifting:
// frozen vectors pin the values against the decoder, and an identity test pins
// the two builders against each other so a future divergence has to be written
// on purpose rather than happening by omission.
TEST_GROUP(PacketFieldEncoding){};

TEST(PacketFieldEncoding, FrozenVectorsMatchTheDecoderContract) {
	const struct {
		double in;
		unsigned int out;
	} SPEED[] = {
		{ -1, 0 }, { 0, 0 }, { 1000, 1 }, { 500000, 127 }, { 1e+06, 127 }, { 1e+08, 127 },
	};
	const struct {
		unsigned int in;
		unsigned int out;
	} BATT[] = {
		{ 0, 0 }, { 2000, 0 }, { 2700, 0 }, { 2701, 0 }, { 2720, 1 }, { 3700, 50 }, { 5240, 127 }, { 6000, 127 },
	};
	const struct {
		double in;
		unsigned int out;
	} LAT[] = {
		{ 0.0, 0 }, { 45.1234, 451233 }, { -45.1234, 1499810 }, { 89.9999, 899999 }, { -89.9999, 1948575 },
	};
	const struct {
		double in;
		unsigned int out;
	} LON[] = {
		{ 0.0, 0 }, { 11.8768, 118768 }, { -33.8232, 2435384 }, { 179.9999, 1799999 }, { -179.9999, 3897151 },
	};
	const struct {
		double in;
		unsigned int out;
	} HEAD[] = {
		{ -5, 0 }, { 0, 0 }, { 90, 63 }, { 180, 126 }, { 359, 252 }, { 400, 254 },
	};
	const struct {
		double in;
		unsigned int out;
	} ALT[] = {
		{ -1000, 0 }, { 0, 0 }, { 40000, 1 }, { 1e+06, 25 }, { 1.016e+07, 254 }, { 9.9e+07, 254 },
	};

	for (auto &v : SPEED)
		CHECK_EQUAL(v.out, PacketField::speed(v.in));
	for (auto &v : BATT)
		CHECK_EQUAL(v.out, PacketField::battery_voltage(v.in));
	for (auto &v : LAT)
		CHECK_EQUAL(v.out, PacketField::latitude(v.in));
	for (auto &v : LON)
		CHECK_EQUAL(v.out, PacketField::longitude(v.in));
	for (auto &v : HEAD)
		CHECK_EQUAL(v.out, PacketField::heading(v.in));
	for (auto &v : ALT)
		CHECK_EQUAL(v.out, PacketField::altitude(v.in));

	// The dead zone is deliberate and the decoder depends on it: everything at
	// or below the 2700 mV reference encodes to 0, and the is_low_battery flag
	// in the header is what distinguishes "flat" from "just below reference".
	CHECK_EQUAL(0U, PacketField::battery_voltage(2000));
	CHECK_EQUAL(0U, PacketField::battery_voltage(2700));

	// 255 is the invalid-fix sentinel for heading and altitude; a valid value
	// must never reach it.
	CHECK_EQUAL(254U, PacketField::heading(1e9));
	CHECK_EQUAL(254U, PacketField::altitude(1e9));
}

TEST(PacketFieldEncoding, ArgosAndLoRaEncodeIdentically) {
	const double D[] = { -179.9999, -45.1234, -0.0001, 0, 0.0001, 11.8768, 90, 179.9999, 1e6, 1e8 };
	for (double x : D) {
		CHECK_EQUAL(ArgosPacketBuilder::convert_speed(x), LoRaPacketBuilder::convert_speed(x));
		CHECK_EQUAL(ArgosPacketBuilder::convert_latitude(x), LoRaPacketBuilder::convert_latitude(x));
		CHECK_EQUAL(ArgosPacketBuilder::convert_longitude(x), LoRaPacketBuilder::convert_longitude(x));
		CHECK_EQUAL(ArgosPacketBuilder::convert_heading(x), LoRaPacketBuilder::convert_heading(x));
		CHECK_EQUAL(ArgosPacketBuilder::convert_altitude(x), LoRaPacketBuilder::convert_altitude(x));
	}
	for (unsigned int mv = 0; mv <= 6000; mv += 7)
		CHECK_EQUAL(ArgosPacketBuilder::convert_battery_voltage(mv), LoRaPacketBuilder::convert_battery_voltage(mv));
}
