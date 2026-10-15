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
#   BUILD_ROOT=/tmp/x ./scripts/build_all_variants.sh
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

WANTED="${1:-}"
GENERATOR=(); command -v ninja >/dev/null && GENERATOR=(-G Ninja)

declare -a NAMES=() RESULTS=() SIZES=()
failed=0

while IFS='|' read -r name flags; do
	case "$name" in ''|'#'*) continue ;; esac
	[ -n "${flags:-}" ] || continue
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
	else
		echo "FAILED"
		# Strip the colour codes first: they sit between the line start and
		# "error:", so a grep for " error:" silently matches nothing.
		sed 's/\x1b\[[0-9;]*[mK]//g' "$dir"/cmake.log "$dir"/build.log 2>/dev/null \
			| grep -E 'error:|Error [0-9]|CMake Error' | head -15 | sed 's/^/      /'
		NAMES+=("$name"); RESULTS+=(FAILED); SIZES+=(-)
		failed=1
	fi
done < "$VARIANT_FILE"

[ "${#NAMES[@]}" -gt 0 ] || { echo "ERROR: no variant matched '${WANTED}'"; exit 1; }

echo
printf '%-18s %-8s %s\n' VARIANT RESULT 'FLASH (text+data)'
for i in "${!NAMES[@]}"; do
	printf '%-18s %-8s %s\n' "${NAMES[$i]}" "${RESULTS[$i]}" "${SIZES[$i]}"
done

# Same table in the GitHub Actions run summary, so a failure is readable
# without opening the log.
if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
	{
		echo "### Firmware compile check"
		echo
		echo "| Variant | Result | Flash (text+data) |"
		echo "|---|---|---|"
		for i in "${!NAMES[@]}"; do
			icon=$([ "${RESULTS[$i]}" = ok ] && echo ':white_check_mark:' || echo ':x:')
			echo "| \`${NAMES[$i]}\` | $icon ${RESULTS[$i]} | ${SIZES[$i]} |"
		done
	} >> "$GITHUB_STEP_SUMMARY"
fi

if [ "$failed" -ne 0 ]; then
	echo
	echo "*** at least one variant does not compile ***"
	exit 1
fi
echo
echo "all ${#NAMES[@]} variants compile"
