#include <cstdint>
#include <vector>

#include "filesystem.hpp"
#include "smd_dfu_image.hpp"

#include "CppUTest/TestHarness.h"


#define BLOCK_COUNT (256)
#define BLOCK_SIZE  (64 * 1024)
#define PAGE_SIZE   (256)

// Size of the SMD image sent on 2026-10-30: not a multiple of SMDSAT_DFU_CHUNK_SIZE.
static constexpr size_t IMAGE_SIZE = 74680;
// The 8-byte [size][crc] header OTAFlashFileUpdater writes ahead of the image.
static constexpr lfs_soff_t HEADER_SIZE = 8;


TEST_GROUP(SmdDfuImage) {
	RamFlash *ram_flash;
	LFSFileSystem *fs;
	std::vector<uint8_t> image;
	uint32_t crc;

	void setup() {
		ram_flash = new RamFlash(BLOCK_COUNT, BLOCK_SIZE, PAGE_SIZE);
		fs = new LFSFileSystem(ram_flash);
		fs->format();
		fs->mount();
		image.resize(IMAGE_SIZE);
		for (size_t i = 0; i < image.size(); i++)
			image[i] = static_cast<uint8_t>(i * 31 + 7);
		crc = spi_crc32_mpeg2(image.data(), image.size());
	}

	void teardown() {
		fs->umount();
		delete fs;
		delete ram_flash;
	}

	void stage(std::vector<uint8_t> content) {
		LFSFile f(fs, "smd_firmware.dat", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
		uint8_t header[HEADER_SIZE] = { 0 };
		f.write(header, sizeof(header));
		f.write(content.data(), content.size());
	}
};


TEST(SmdDfuImage, Crc32Mpeg2CheckValue) {
	const uint8_t check[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	UNSIGNED_LONGS_EQUAL(0x0376E6E7, spi_crc32_mpeg2(check, sizeof(check)));
}

TEST(SmdDfuImage, ChunkedCrcEqualsOneShot) {
	uint32_t c = 0xFFFFFFFF;
	c = spi_crc32_mpeg2_update(c, image.data(), 1000);
	c = spi_crc32_mpeg2_update(c, image.data() + 1000, image.size() - 1000);
	UNSIGNED_LONGS_EQUAL(crc, c);
}

TEST(SmdDfuImage, IntactImageIsAcceptedAndThePositionKept) {
	stage(image);
	LFSFile f(fs, "smd_firmware.dat", LFS_O_RDONLY);
	f.seek(HEADER_SIZE);
	CHECK_TRUE(smd_dfu_staged_image_ok(f, image.size(), crc));
	LONGS_EQUAL(HEADER_SIZE, f.tell());
}

TEST(SmdDfuImage, OneDamagedByteIsRefused) {
	std::vector<uint8_t> damaged = image;
	damaged[40000] ^= 0x01;
	stage(damaged);
	LFSFile f(fs, "smd_firmware.dat", LFS_O_RDONLY);
	f.seek(HEADER_SIZE);
	CHECK_FALSE(smd_dfu_staged_image_ok(f, image.size(), crc));
	LONGS_EQUAL(HEADER_SIZE, f.tell());
}

TEST(SmdDfuImage, ShortImageIsRefused) {
	stage(std::vector<uint8_t>(image.begin(), image.end() - 100));
	LFSFile f(fs, "smd_firmware.dat", LFS_O_RDONLY);
	f.seek(HEADER_SIZE);
	CHECK_FALSE(smd_dfu_staged_image_ok(f, image.size(), crc));
	LONGS_EQUAL(HEADER_SIZE, f.tell());
}
