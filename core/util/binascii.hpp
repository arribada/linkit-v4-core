/**
 * @file binascii.hpp
 * @brief Hex encode/decode — hexlify (binary→hex string) and unhexlify (hex string→binary).
 */

#pragma once

#include <string>

/// @brief Binary ↔ hex string conversion utilities.
class Binascii {
public:
	/// @brief Convert binary data to uppercase hex string.
	/// @param input  Binary data.
	/// @return Hex string (2 chars per byte, uppercase).
	static std::string hexlify(std::string input) {
		static const char hex_digits[] = "0123456789ABCDEF";

		std::string output;
		output.reserve(input.length() * 2);
		for (unsigned char c : input) {
			output.push_back(hex_digits[c >> 4]);
			output.push_back(hex_digits[c & 0xF]);
		}
		return output;
	}

	/// @brief Convert hex string to binary data. Supports uppercase and lowercase hex.
	/// @param buffer  Hex string (must be even length).
	/// @return Binary data, or empty string if input length is odd.
	static std::string unhexlify(std::string buffer) {
		unsigned int length = buffer.size();
		unsigned int i = 0;

		std::string output;
		if (length % 2) return output;

		output.reserve(buffer.size() / 2);

		while (length) {
			char high = buffer[i++];
			high = (high >= 'a') ? (high - 'a' + 10) : (high >= 'A') ? (high - 'A' + 10) : (high - '0');
			char low = buffer[i++];
			low = (low >= 'a') ? (low - 'a' + 10) : (low >= 'A') ? (low - 'A' + 10) : (low - '0');
			output.push_back((high << 4) | (low & 0xF));
			length -= 2;
		}

		return output;
	}
};
