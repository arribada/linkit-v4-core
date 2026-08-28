#include "CppUTest/TestHarness.h"
#include "binascii.hpp"

using namespace std::literals::string_literals;

TEST_GROUP(Binascii){};

TEST(Binascii, HexlifyBasic) {
	std::string input = "\x01\x02\x0A\xFF"s;
	STRCMP_EQUAL("01020AFF", Binascii::hexlify(input).c_str());
}

TEST(Binascii, HexlifyEmptyString) {
	STRCMP_EQUAL("", Binascii::hexlify("").c_str());
}

TEST(Binascii, HexlifySingleByte) {
	STRCMP_EQUAL("00", Binascii::hexlify("\x00"s).c_str());
	STRCMP_EQUAL("FF", Binascii::hexlify("\xFF"s).c_str());
	STRCMP_EQUAL("7F", Binascii::hexlify("\x7F"s).c_str());
}

TEST(Binascii, UnhexlifyBasic) {
	std::string result = Binascii::unhexlify("01020AFF");
	CHECK_EQUAL(4U, result.size());
	CHECK_EQUAL(0x01, (unsigned char)result[0]);
	CHECK_EQUAL(0x02, (unsigned char)result[1]);
	CHECK_EQUAL(0x0A, (unsigned char)result[2]);
	CHECK_EQUAL(0xFF, (unsigned char)result[3]);
}

TEST(Binascii, UnhexlifyEmptyString) {
	STRCMP_EQUAL("", Binascii::unhexlify("").c_str());
}

TEST(Binascii, UnhexlifyOddLength) {
	// Odd-length hex string should return empty
	STRCMP_EQUAL("", Binascii::unhexlify("ABC").c_str());
}

TEST(Binascii, RoundTrip) {
	std::string original = "\xDE\xAD\xBE\xEF"s;
	std::string hex = Binascii::hexlify(original);
	std::string result = Binascii::unhexlify(hex);
	STRCMP_EQUAL(original.c_str(), result.c_str());
	CHECK_EQUAL(original.size(), result.size());
}

TEST(Binascii, HexlifyAllBytes) {
	// Test all 256 byte values
	std::string all_bytes;
	all_bytes.reserve(256);
	for (int i = 0; i < 256; i++)
		all_bytes.push_back((char)i);

	std::string hex = Binascii::hexlify(all_bytes);
	CHECK_EQUAL(512U, hex.size());

	std::string roundtrip = Binascii::unhexlify(hex);
	CHECK_EQUAL(256U, roundtrip.size());
	for (int i = 0; i < 256; i++)
		CHECK_EQUAL((unsigned char)i, (unsigned char)roundtrip[i]);
}
