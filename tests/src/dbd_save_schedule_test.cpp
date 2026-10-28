// When the M10Q driver writes its fetched navigation database (MGA-DBD) to
// flash: not right after the fix, at a quiet moment, and never later than a
// fixed delay after the first fix still unsaved.

#include "m10qasync/dbd_save_schedule.hpp"

#include "CppUTest/TestHarness.h"

TEST_GROUP(DbdSaveSchedule){};

// The first fix of a window is written at the maximum delay, unless something
// quieter (a dive) or a power-down comes first.
TEST(DbdSaveSchedule, FirstFixIsDueAtTheMaximumDelay) {
	DbdSaveSchedule s;
	CHECK_FALSE(s.unsaved());
	CHECK_EQUAL(1000U + DbdSaveSchedule::MAX_DELAY_MS, s.on_fetched(1000));
	CHECK_TRUE(s.unsaved());
}

// A later fix replaces the data, never the deadline: with a session every two
// minutes, a deadline reset at each fix would never be reached.
TEST(DbdSaveSchedule, LaterFixesNeverPushTheDeadlineBack) {
	DbdSaveSchedule s;
	const uint64_t due = s.on_fetched(0);
	CHECK_EQUAL(due, s.on_fetched(120000));
	CHECK_EQUAL(due, s.on_fetched(240000));
}

// Past the deadline, a fix is still written no sooner than the minimum delay
// after it: right after a fix, the scheduler is sending the position frame.
TEST(DbdSaveSchedule, NeverSoonerThanTheMinimumAfterAFix) {
	DbdSaveSchedule s;
	s.on_fetched(0);
	const uint64_t late = DbdSaveSchedule::MAX_DELAY_MS + 1000;
	CHECK_EQUAL(late + DbdSaveSchedule::MIN_DELAY_MS, s.on_fetched(late));
}

// Once on flash, the next fix opens a new window.
TEST(DbdSaveSchedule, SavedOpensANewWindow) {
	DbdSaveSchedule s;
	s.on_fetched(0);
	s.saved();
	CHECK_FALSE(s.unsaved());
	CHECK_EQUAL(500000U + DbdSaveSchedule::MAX_DELAY_MS, s.on_fetched(500000));
}
