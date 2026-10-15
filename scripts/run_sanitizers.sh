#!/bin/bash
# Run the host test suite under AddressSanitizer and UndefinedBehaviorSanitizer.
#
# WHY THIS EXISTS AS A SCRIPT. Nothing in the repo had ever been run under a
# sanitizer -- no ASan, no UBSan, no valgrind, no flag anywhere in
# tests/CMakeLists.txt or either workflow. The recipe is three lines, but two of
# them are not guessable, and without them the run is either impossible or
# misread. Both are written down below, because re-deriving them cost an
# afternoon.
#
# GOTCHA 1: IT NEEDS ASLR OFF. Under WSL2 the ASan-instrumented binary dies
# instantly with nothing but a row of `AddressSanitizer:DEADLYSIGNAL` -- no test
# output, no stack, exit 139. That is the known shadow-memory/ASLR collision, not
# a defect in this code: `setarch -R` and the same binary runs clean. Anyone
# hitting the DEADLYSIGNAL without knowing this will go looking for a crash that
# is not there.
#
# GOTCHA 2: THE UBSAN ALIGNMENT REPORTS ARE A HOST ARTEFACT. A clean run still
# prints about 50 "misaligned address ... requires 16 byte alignment" lines, all
# of one family, centred on Scheduler::Task and the inplace_function inside it.
# They cannot happen in any shipped binary, for two independent reasons, both
# measured:
#
#   - inplace_function's alignment comes from a union that includes long double
#     only when the capacity is large enough to hold it. tests/CMakeLists.txt
#     builds with INPLACE_FUNCTION_SIZE_SCHEDULER=1024; the firmware default is
#     12 (core/scheduling/scheduler.hpp:46). On x86-64 that is alignof 16 versus
#     alignof 8.
#   - and on the target it is 8 either way, because arm-none-eabi gives
#     long double the alignment of double.
#
#         alignof(inplace_function<void(),Cap>)   Cap=1024   Cap=12
#         x86-64 host                                  16        8
#         ARM Cortex-M4                                 8        8
#
#     So the combination UBSan complains about -- 16-byte requirement -- exists
#     only in the host test build. Do not "fix" the scheduler over these lines.
#     The remaining reports in tests/src/logger_test.cpp are the test writing an
#     unsigned int into a packed buffer, also test-only.
#
# WHAT A PASS LOOKS LIKE. Zero `ERROR: AddressSanitizer` (memory errors), and
# the CppUTest verdict OK. Measured on this tree: 898 tests, 96276 checks, zero
# memory errors, ~58 s.
#
#   ./scripts/run_sanitizers.sh            # the default suite
#   ./scripts/run_sanitizers.sh -sg Sm     # any TrackerTests argument
#
# Not wired into CI: the build is a full rebuild with instrumentation (a ~245 MB
# binary) and the run is twice the wall clock. It is a thing to run before a
# deployment, by hand, which is what the recipe being written down is for.

set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${SAN_BUILD_DIR:-/tmp/linkit-san}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"

# Default to what the CI unit job runs, minus the SWS groups (minutes each).
ARGS=("$@")
[ "${#ARGS[@]}" -gt 0 ] || ARGS=(-xg SWS)

command -v setarch >/dev/null || { echo "ERROR: setarch not found (util-linux); see GOTCHA 1"; exit 1; }

SAN="-fsanitize=address,undefined -fno-omit-frame-pointer -g"

echo ">>> configuring $BUILD"
mkdir -p "$BUILD"
cmake -S "$REPO/tests" -B "$BUILD" -G Ninja \
	-DCMAKE_BUILD_TYPE=Debug \
	-DCMAKE_C_FLAGS="$SAN" \
	-DCMAKE_CXX_FLAGS="$SAN" \
	-DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined" >"$BUILD/cmake.log" 2>&1 ||
	{ tail -20 "$BUILD/cmake.log"; exit 1; }

echo ">>> building (instrumented, slow)"
cmake --build "$BUILD" --target TrackerTests -j "$JOBS" >"$BUILD/build.log" 2>&1 ||
	{ sed 's/\x1b\[[0-9;]*[mK]//g' "$BUILD/build.log" | grep -E 'error:' | head -20; exit 1; }

# From tests/, or the four PASPW cases fail on a relative path to data/*.json
# and send you hunting a defect that is not there (.claude/CLAUDE.md).
echo ">>> running ${ARGS[*]}"
log="$BUILD/run.log"
(
	cd "$REPO/tests" || exit 1
	ASAN_OPTIONS=detect_leaks=0:abort_on_error=0:print_stacktrace=1 \
		UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=0 \
		setarch -R "$BUILD/TrackerTests" "${ARGS[@]}"
) >"$log" 2>&1
rc=$?

echo
verdict=$(grep -E '^(OK|Errors)' "$log" | tail -1)
mem=$(grep -c 'ERROR: AddressSanitizer' "$log")
align=$(grep -c 'runtime error' "$log")
echo "verdict:            ${verdict:-<none: see $log>}"
echo "ASan memory errors: $mem"
echo "UBSan reports:      $align  (see GOTCHA 2 before acting on alignment ones)"
if [ "$align" -gt 0 ]; then
	echo "  families:"
	grep -oE 'runtime error: [a-z ]+' "$log" | sort | uniq -c | sed 's/^/    /'
	echo "  files:"
	grep -E 'runtime error' "$log" | sed "s|.*/linkit-v4-core/||;s|:[0-9]*:[0-9]*:.*||" | sort | uniq -c | sed 's/^/    /'
fi
echo "full log:           $log"

if [ "$mem" -gt 0 ]; then
	echo
	echo "*** AddressSanitizer found memory errors — those are never a host artefact ***"
	exit 1
fi
[ "$rc" -eq 0 ] || { echo; echo "*** the suite itself failed (exit $rc) ***"; exit "$rc"; }
echo
echo "no memory errors, suite green"
