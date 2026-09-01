#!/bin/bash
# Run unit tests with summary report

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/tests/build"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    LinkIt V4 Unit Tests Runner${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Build tests. ALWAYS, not just when the binary is missing.
#
# This used to be `if [ ! -f TrackerTests ]`, which meant every run after the
# first one tested whatever binary happened to be lying in tests/build — source
# changes included. A green run then proved nothing about the code just written,
# and said so in a reassuring voice. Caught 2026-09-01 after an EXTINT change
# reported 772/772 while actually breaking 18 tests; ninja is incremental, so
# always configuring and building costs nothing when nothing changed.
echo -e "${YELLOW}Building tests...${NC}"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    cmake -GNinja ..
fi
if ! ninja; then
    echo -e "${RED}Test build FAILED — not running a stale binary${NC}"
    exit 1
fi
echo ""
cd "$PROJECT_DIR"

cd "$BUILD_DIR"

# Run tests and capture output
echo -e "${BLUE}Running unit tests...${NC}"
echo ""

# Run tests with verbose output and capture exit code
# -xg SWS: exclude SWSAnalog and SWS groups (long-running underwater simulation tests)
#   Run them separately with: ./TrackerTests -g SWSAnalog  or  -g SWS
# -p: run each test in a separate process to avoid cross-test memory corruption
set +e
OUTPUT=$("./TrackerTests" -v -p -xg SWS 2>&1)
EXIT_CODE=$?
set -e

# Display the output
echo "$OUTPUT"

# Parse results from CppUTest output
# CppUTest format: "OK (X tests, Y ran, Z checks, W ignored, V filtered out, Xms)"
# or "Errors (X failures, Y tests, Z ran, ...)"

# Extract numbers from the summary line
SUMMARY_LINE=$(echo "$OUTPUT" | grep -E '^(OK|Errors) \(' | tail -1)

if echo "$SUMMARY_LINE" | grep -q "^OK"; then
    # Success case: OK (X tests, Y ran, Z checks, W ignored, V filtered out, Xms)
    TOTAL_TESTS=$(echo "$SUMMARY_LINE" | grep -oP '\d+ tests' | grep -oP '\d+' | head -1)
    FAILURES=0
elif echo "$SUMMARY_LINE" | grep -q "^Errors"; then
    # Failure case: Errors (X failures, Y tests, Z ran, ...)
    FAILURES=$(echo "$SUMMARY_LINE" | grep -oP '\d+ failures' | grep -oP '\d+' | head -1)
    TOTAL_TESTS=$(echo "$SUMMARY_LINE" | grep -oP '\d+ tests' | grep -oP '\d+' | head -1)
else
    TOTAL_TESTS=0
    FAILURES=0
fi

# Default values if parsing failed
if [ -z "$TOTAL_TESTS" ]; then
    TOTAL_TESTS=0
fi
if [ -z "$FAILURES" ]; then
    FAILURES=0
fi

PASSED=$((TOTAL_TESTS - FAILURES))

# Calculate pass rate using awk (more portable than bc)
if [ "$TOTAL_TESTS" -gt 0 ]; then
    PASS_RATE=$(awk "BEGIN {printf \"%.1f\", $PASSED * 100 / $TOTAL_TESTS}")
else
    PASS_RATE="0.0"
fi

# Print summary
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}         TEST SUMMARY REPORT${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "  Total Tests:  ${YELLOW}$TOTAL_TESTS${NC}"
echo -e "  Passed:       ${GREEN}$PASSED${NC}"
echo -e "  Failed:       ${RED}$FAILURES${NC}"
echo -e "  Pass Rate:    ${YELLOW}$PASS_RATE%${NC}"
echo -e "${BLUE}========================================${NC}"

if [ "$EXIT_CODE" -eq 0 ]; then
    echo -e "  Status:       ${GREEN}ALL TESTS PASSED${NC}"
else
    echo -e "  Status:       ${RED}SOME TESTS FAILED${NC}"
fi
echo -e "${BLUE}========================================${NC}"

exit $EXIT_CODE
