/**
 * @file lora_packet_builder_motion.cpp
 * @brief LoRaPacketBuilder — MOTION v1 debug block (LORA_MOTION_EXT, Cyprus boat build).
 *
 * A translation unit of its own on purpose, compiled into the firmware only
 * when LORA_MOTION_EXT is on. Placed in lora_packet_builder.cpp, the unused
 * std::string::resize() below still changed how GCC outlined
 * std::string::assign() for the existing builders in that file: the plain
 * LoRa firmware was no longer byte-identical, even with these two functions
 * garbage-collected.
 */

#include "lora_packet_builder.hpp"
#include "bitpack.hpp"

uint8_t LoRaPacketBuilder::encode_minutes_since(std::time_t now, std::time_t then) {
	if (now == 0 || then == 0 || now < then) return MOTION_MINUTES_UNKNOWN;
	const std::time_t minutes = (now - then) / 60;
	return (minutes >= MOTION_MINUTES_SATURATED) ? MOTION_MINUTES_SATURATED : (uint8_t)minutes;
}

void LoRaPacketBuilder::append_motion_ext(KineisPacket &packet, unsigned int &size_bits, const LoRaMotionExt &motion) {
	// Start on the next byte boundary, so the block is always the last
	// MOTION_EXT_BYTES of the payload whatever padding the base frame carries.
	unsigned int base_pos = packet.size() * BITS_PER_BYTE;
	packet.resize(packet.size() + MOTION_EXT_BYTES, 0);

	PACK_BITS(MOTION_EXT_TAG, packet, base_pos, 4);
	PACK_BITS(motion.moored ? 1U : 0U, packet, base_pos, 1);
	PACK_BITS(motion.axl_holdoff ? 1U : 0U, packet, base_pos, 1);
	PACK_BITS(0U, packet, base_pos, 2);  // reserved
	PACK_BITS((unsigned int)motion.wakeups, packet, base_pos, 8);
	PACK_BITS((unsigned int)motion.min_since_wakeup, packet, base_pos, 8);
	PACK_BITS((unsigned int)motion.min_since_axl_exit, packet, base_pos, 8);

	size_bits = base_pos;
}
