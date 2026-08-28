#include "CppUTest/TestHarness.h"
#include "haversine.hpp"
#include <cmath>

TEST_GROUP(Haversine){};

TEST(Haversine, SamePointZeroDistance) {
	double d = haversine_distance(0.0, 0.0, 0.0, 0.0);
	DOUBLES_EQUAL(0.0, d, 0.001);
}

TEST(Haversine, SamePointNonZero) {
	double d = haversine_distance(2.3522, 48.8566, 2.3522, 48.8566);  // Paris
	DOUBLES_EQUAL(0.0, d, 0.001);
}

TEST(Haversine, ParisToLondon) {
	// Paris (48.8566, 2.3522) to London (51.5074, -0.1278)
	// Known distance ~343 km
	double d = haversine_distance(2.3522, 48.8566, -0.1278, 51.5074);
	DOUBLES_EQUAL(343.5, d, 5.0);  // Allow 5km tolerance
}

TEST(Haversine, NewYorkToLosAngeles) {
	// New York (40.7128, -74.0060) to Los Angeles (34.0522, -118.2437)
	// Known distance ~3944 km
	double d = haversine_distance(-74.0060, 40.7128, -118.2437, 34.0522);
	DOUBLES_EQUAL(3944.0, d, 20.0);  // Allow 20km tolerance
}

TEST(Haversine, EquatorPoints) {
	// Two points on equator, 1 degree apart
	// At equator, 1 degree longitude ~ 111.32 km
	double d = haversine_distance(0.0, 0.0, 1.0, 0.0);
	DOUBLES_EQUAL(111.32, d, 1.0);
}

TEST(Haversine, PoleToEquator) {
	// North pole (90, 0) to equator (0, 0)
	// Quarter of earth circumference ~ 10008 km
	double d = haversine_distance(0.0, 90.0, 0.0, 0.0);
	DOUBLES_EQUAL(10008.0, d, 20.0);
}

TEST(Haversine, Antipodes) {
	// Point to its antipode = half circumference ~ 20015 km
	double d = haversine_distance(0.0, 0.0, 180.0, 0.0);
	DOUBLES_EQUAL(20015.0, d, 100.0);
}

TEST(Haversine, SymmetricDistance) {
	double d1 = haversine_distance(2.3522, 48.8566, -0.1278, 51.5074);
	double d2 = haversine_distance(-0.1278, 51.5074, 2.3522, 48.8566);
	DOUBLES_EQUAL(d1, d2, 0.001);
}
