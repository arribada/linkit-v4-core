#!/bin/bash
# Compile every firmware configuration in scripts/firmware_variants.txt.
#
# Answers one question: does the tree still compile and link for every board
# and comm variant we ship, at every DEBUG_LEVEL? It does NOT merge a
# bootloader, sign, or flash -- scripts/build_*.sh do that, and they need the
# unversioned build_config.sh. This one calls cmake directly, like
# .github/workflows/build.yml, so it runs on a bare CI runner and inside a
# git worktree (where build_config.sh is absent -- see .claude/CLAUDE.md).
#
#   ./scripts/build_all_variants.sh                 # all variants
#   ./scripts/build_all_variants.sh linkit-smd-uart # one, by name
#   BUILD_ROOT=/tmp/x ./scripts/build_all_variants.sh   # a build root of your own
#
# Two runs sharing a BUILD_ROOT serialise on a lock file; give a second run its
# own BUILD_ROOT if you want them to go in parallel.
#
# Keeps going after a failure and reports every variant, because stopping at
# the first hides how much is broken. Exits non-zero if any failed.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PORT_DIR="$PROJECT_ROOT/ports/nrf52840"
VARIANT_FILE="${VARIANT_FILE:-$SCRIPT_DIR/firmware_variants.txt}"
BUILD_ROOT="${BUILD_ROOT:-$PORT_DIR/build/ci}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"

[ -f "$VARIANT_FILE" ] || { echo "ERROR: $VARIANT_FILE not found"; exit 1; }
command -v arm-none-eabi-gcc >/dev/null || { echo "ERROR: arm-none-eabi-gcc not in PATH"; exit 1; }
# The CMake post-build step packages a DFU zip with nrfutil; without it every
# variant fails at the last step of an otherwise successful compile.
command -v nrfutil >/dev/null || { echo "ERROR: nrfutil not in PATH"; exit 1; }

# One builder per build root. Each variant directory is rm -rf'd before use, so
# a second concurrent run against the same BUILD_ROOT deletes the first one's
# tree mid-compile and reports a failure that is not real. That is not
# hypothetical: a parallel reviewer running this script made a clean tree report
# "at least one variant does not compile", and the run right after it was green.
# CI runs this alone, so the lock only ever matters locally -- which is exactly
# where the false result is most expensive, because it sends someone hunting.
mkdir -p "$BUILD_ROOT"
exec 9>"$BUILD_ROOT/.lock"
if command -v flock >/dev/null 2>&1; then
	if ! flock -n 9; then
		echo "another build already holds $BUILD_ROOT — waiting for it to finish"
		flock 9
	fi
fi

WANTED="${1:-}"
GENERATOR=(); command -v ninja >/dev/null && GENERATOR=(-G Ninja)

declare -a NAMES=() RESULTS=() SIZES=()
failed=0

# The GitHub run summary is written row by row, as each variant finishes, not
# once at the end: a job that times out or is cancelled mid-loop would
# otherwise leave no table at all, which is exactly when you want to know how
# far it got.
summary() { [ -n "${GITHUB_STEP_SUMMARY:-}" ] && printf '%s\n' "$1" >> "$GITHUB_STEP_SUMMARY"; return 0; }
summary "### Firmware compile check"
summary ""
summary "| Variant | Result | Flash (text+data) |"
summary "|---|---|---|"

while IFS='|' read -r name flags; do
	case "$name" in ''|'#'*) continue ;; esac
	# A line with a name but no flags is a MALFORMED variant, not a comment.
	# Skipping it quietly meant the run still printed "all N variants compile"
	# and exited 0 with that variant never built -- a typo in the list would
	# have silently removed a board from the gate.
	if [ -z "${flags:-}" ]; then
		echo "ERROR: $VARIANT_FILE: variant '$name' has no cmake flags (missing '|'?)"
		exit 1
	fi
	[ -z "$WANTED" ] || [ "$WANTED" = "$name" ] || continue

	dir="$BUILD_ROOT/$name"
	rm -rf "$dir" && mkdir -p "$dir"
	printf '>>> %-18s ' "$name"

	ok=true
	( cd "$dir" \
		&& cmake "${GENERATOR[@]}" \
			-DCMAKE_TOOLCHAIN_FILE="$PORT_DIR/toolchain_arm_gcc_nrf52.cmake" \
			-DCMAKE_BUILD_TYPE=Release $flags "$PORT_DIR" >cmake.log 2>&1 \
		&& cmake --build . -j "$JOBS" >build.log 2>&1 ) || ok=false

	elf=$(ls "$dir"/LinkIt_*.elf 2>/dev/null | head -1)
	if [ "$ok" = true ] && [ -n "$elf" ]; then
		# text+data is what occupies flash; a jump here is worth a look even
		# when the build passed.
		sz=$(arm-none-eabi-size "$elf" | awk 'NR==2 {print $1 + $2}')
		echo "ok    flash ${sz} B"
		NAMES+=("$name"); RESULTS+=(ok); SIZES+=("$sz")
		summary "| \`$name\` | :white_check_mark: ok | $sz |"
	else
		echo "FAILED"
		# Strip the colour codes first: they sit between the line start and
		# "error:", so a grep for " error:" silently matches nothing.
		sed 's/\x1b\[[0-9;]*[mK]//g' "$dir"/cmake.log "$dir"/build.log 2>/dev/null \
			| grep -E 'error:|Error [0-9]|CMake Error' | head -15 | sed 's/^/      /'
		NAMES+=("$name"); RESULTS+=(FAILED); SIZES+=(-)
		# The first error line in the summary too: a configure-time failure
		# otherwise left nothing readable in CI without opening the log.
		first_err=$(sed 's/\x1b\[[0-9;]*[mK]//g' "$dir"/cmake.log "$dir"/build.log 2>/dev/null \
			| grep -E 'error:|Error [0-9]|CMake Error' | head -1 | tr -d '|')
		summary "| \`$name\` | :x: FAILED | ${first_err:-see log} |"
		failed=1
	fi
done < "$VARIANT_FILE"

[ "${#NAMES[@]}" -gt 0 ] || { echo "ERROR: no variant matched '${WANTED}'"; exit 1; }

echo
printf '%-18s %-8s %s\n' VARIANT RESULT 'FLASH (text+data)'
for i in "${!NAMES[@]}"; do
	printf '%-18s %-8s %s\n' "${NAMES[$i]}" "${RESULTS[$i]}" "${SIZES[$i]}"
done

if [ "$failed" -ne 0 ]; then
	echo
	echo "*** at least one variant does not compile ***"
	exit 1
fi
echo
echo "all ${#NAMES[@]} variants compile"
