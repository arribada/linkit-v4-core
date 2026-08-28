#include "CppUTest/TestHarness.h"
#include "bitpack.hpp"

using namespace std::literals::string_literals;

TEST_GROUP(BitPack){};

TEST(BitPack, ExtractSingleByte) {
	std::string data = "\xA5"s;  // 10100101
	CHECK_EQUAL(0xA5U, extract_bits(data, 0, 8));
}

TEST(BitPack, ExtractHighNibble) {
	std::string data = "\xA5"s;  // 1010 0101
	CHECK_EQUAL(0x0AU, extract_bits(data, 0, 4));
}

TEST(BitPack, ExtractLowNibble) {
	std::string data = "\xA5"s;  // 1010 0101
	CHECK_EQUAL(0x05U, extract_bits(data, 4, 4));
}

TEST(BitPack, ExtractCrossByteBoundary) {
	std::string data = "\xAB\xCD"s;  // 10101011 11001101
	// Extract 8 bits starting at bit 4: 1011 1100 = 0xBC
	CHECK_EQUAL(0xBCU, extract_bits(data, 4, 8));
}

TEST(BitPack, Extract16Bits) {
	std::string data = "\xAB\xCD"s;
	CHECK_EQUAL(0xABCDU, extract_bits(data, 0, 16));
}

TEST(BitPack, ExtractSingleBit) {
	std::string data = "\x80"s;  // 10000000
	CHECK_EQUAL(1U, extract_bits(data, 0, 1));
	CHECK_EQUAL(0U, extract_bits(data, 1, 1));
}

TEST(BitPack, Extract32Bits) {
	std::string data = "\x12\x34\x56\x78"s;
	CHECK_EQUAL(0x12345678U, extract_bits(data, 0, 32));
}

TEST(BitPack, ExtractWithMacro) {
	std::string data = "\xAB\xCD\xEF"s;
	uint32_t val;
	int start = 0;
	EXTRACT_BITS(val, data, start, 8);
	CHECK_EQUAL(0xABU, val);
	CHECK_EQUAL(8, start);
	EXTRACT_BITS(val, data, start, 8);
	CHECK_EQUAL(0xCDU, val);
	CHECK_EQUAL(16, start);
}

TEST(BitPack, PackSingleByte) {
	std::string output = "\x00"s;
	pack_bits(output, 0xA5, 0, 8);
	CHECK_EQUAL((char)0xA5, output[0]);
}

TEST(BitPack, PackHighNibble) {
	std::string output = "\x00"s;
	pack_bits(output, 0x0A, 0, 4);
	CHECK_EQUAL((char)0xA0, output[0]);
}

TEST(BitPack, PackCrossByteBoundary) {
	std::string output = "\x00\x00"s;
	pack_bits(output, 0xBC, 4, 8);  // Pack 0xBC at bit offset 4
	// Byte 0: 0000 1011 = 0x0B, Byte 1: 1100 0000 = 0xC0
	CHECK_EQUAL((char)0x0B, output[0]);
	CHECK_EQUAL((char)0xC0, output[1]);
}

TEST(BitPack, PackAndExtractRoundTrip) {
	std::string buffer = "\x00\x00\x00\x00"s;
	int pack_pos = 0;
	PACK_BITS(0x1F, buffer, pack_pos, 5);
	PACK_BITS(0x2A, buffer, pack_pos, 7);
	PACK_BITS(0x0FF, buffer, pack_pos, 12);

	int extract_pos = 0;
	uint32_t v1, v2, v3;
	EXTRACT_BITS(v1, buffer, extract_pos, 5);
	EXTRACT_BITS(v2, buffer, extract_pos, 7);
	EXTRACT_BITS(v3, buffer, extract_pos, 12);
	CHECK_EQUAL(0x1FU, v1);
	CHECK_EQUAL(0x2AU, v2);
	CHECK_EQUAL(0x0FFU, v3);
}

TEST(BitPack, Pack16Bits) {
	std::string output = "\x00\x00"s;
	pack_bits(output, 0xABCD, 0, 16);
	CHECK_EQUAL((char)0xAB, output[0]);
	CHECK_EQUAL((char)0xCD, output[1]);
}
