#!/bin/bash
# =============================================================================
# LinkIt Build Environment Setup Script
# =============================================================================
# This script detects, installs, and configures the build environment.
# It will:
#   1. Install missing tools (apt, pip, Nordic downloads)
#   2. Detect tool paths
#   3. Generate build_config.sh
#   4. Update .vscode/tasks.json
#   5. Update SDK Makefile.posix
#   6. Verify the configuration
#
# Usage:
#   ./scripts/setup_environment.sh            # interactive (asks before install)
#   ./scripts/setup_environment.sh --auto     # auto-install everything
# =============================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parse arguments
AUTO_INSTALL=false
if [ "$1" = "--auto" ] || [ "$1" = "-y" ]; then
    AUTO_INSTALL=true
fi

# Helper: ask yes/no (auto-yes if --auto)
ask_yes_no() {
    if [ "$AUTO_INSTALL" = true ]; then
        return 0
    fi
    read -p "$1 [Y/n] " answer
    case "$answer" in
        [nN]*) return 1 ;;
        *) return 0 ;;
    esac
}

echo -e "${BLUE}=========================================================================${NC}"
echo -e "${BLUE}LinkIt Build Environment Setup${NC}"
echo -e "${BLUE}=========================================================================${NC}"
echo ""

# =============================================================================
# Step 1: Install missing tools
# =============================================================================
echo -e "${BLUE}Step 1: Installing missing tools${NC}"
echo ""

APT_PACKAGES_NEEDED=()

# --- ARM GCC Toolchain (downloaded from ARM, NOT from apt) ---
# The apt version (gcc-arm-none-eabi) ships GCC 13.x which triggers false
# positive warnings in nRF5 SDK 17 and ETL. ARM GCC 10.3 is the recommended
# version for nRF5 SDK 17.
ARM_GCC_VERSION="10.3-2021.10"
ARM_GCC_DIR="$HOME/tools/gcc-arm-none-eabi-${ARM_GCC_VERSION}"
ARM_GCC_URL="https://developer.arm.com/-/media/Files/downloads/gnu-rm/${ARM_GCC_VERSION}/gcc-arm-none-eabi-${ARM_GCC_VERSION}-x86_64-linux.tar.bz2"

if [ -f "$ARM_GCC_DIR/bin/arm-none-eabi-gcc" ]; then
    echo -e "${GREEN}  [OK] arm-none-eabi-gcc ($(${ARM_GCC_DIR}/bin/arm-none-eabi-gcc -dumpversion) in $ARM_GCC_DIR)${NC}"
else
    echo -e "${YELLOW}  [MISSING] ARM GCC Toolchain ${ARM_GCC_VERSION}${NC}"
    if ask_yes_no "Download ARM GCC ${ARM_GCC_VERSION} to ~/tools/?"; then
        mkdir -p "$HOME/tools"
        TMPFILE=$(mktemp /tmp/gcc-arm-XXXXXX.tar.bz2)
        echo -e "${BLUE}  Downloading ARM GCC ${ARM_GCC_VERSION} (~130MB)...${NC}"
        if curl -fSL -o "$TMPFILE" "$ARM_GCC_URL" 2>/dev/null || \
           wget -q --show-progress -O "$TMPFILE" "$ARM_GCC_URL" 2>/dev/null; then
            echo -e "${BLUE}  Extracting to ~/tools/...${NC}"
            tar -xjf "$TMPFILE" -C "$HOME/tools/"
            rm -f "$TMPFILE"
            if [ -f "$ARM_GCC_DIR/bin/arm-none-eabi-gcc" ]; then
                echo -e "${GREEN}  ARM GCC ${ARM_GCC_VERSION} installed successfully${NC}"
            else
                echo -e "${RED}  Extraction failed - check ~/tools/${NC}"
            fi
        else
            echo -e "${RED}  Download failed${NC}"
            echo -e "${YELLOW}  Download manually from: https://developer.arm.com/downloads/-/gnu-rm${NC}"
            rm -f "$TMPFILE"
        fi
    else
        echo -e "${YELLOW}  Skipping ARM GCC (build will use system version if available)${NC}"
    fi
fi
echo ""

# --- Check apt packages ---
check_apt_tool() {
    local cmd="$1"
    local pkg="$2"
    local desc="$3"
    if ! command -v "$cmd" &> /dev/null; then
        echo -e "${YELLOW}  [MISSING] $cmd ($desc)${NC}"
        APT_PACKAGES_NEEDED+=("$pkg")
    else
        echo -e "${GREEN}  [OK] $cmd${NC}"
    fi
}

check_apt_tool "make"              "build-essential"     "build system"
check_apt_tool "cmake"             "cmake"              "build generator (unit tests)"
check_apt_tool "ninja"             "ninja-build"        "fast build system (simulations)"
check_apt_tool "crc32"             "libarchive-zip-perl" "CRC32 tool (firmware .img)"
check_apt_tool "xxd"               "xxd"                "hex dump tool (firmware .img)"

echo ""

# --- Install apt packages ---
if [ ${#APT_PACKAGES_NEEDED[@]} -gt 0 ]; then
    echo -e "${YELLOW}Missing apt packages: ${APT_PACKAGES_NEEDED[*]}${NC}"
    if ask_yes_no "Install with apt?"; then
        echo -e "${BLUE}  Running: sudo apt-get install -y ${APT_PACKAGES_NEEDED[*]}${NC}"
        sudo apt-get update -qq && sudo apt-get install -y "${APT_PACKAGES_NEEDED[@]}"
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}  apt packages installed successfully${NC}"
        else
            echo -e "${RED}  apt install failed - you may need to install manually${NC}"
        fi
    else
        echo -e "${YELLOW}  Skipping apt packages${NC}"
    fi
    echo ""
fi

# --- Install nrfutil (Nordic CLI + nrf5sdk-tools plugin) ---
# The new nrfutil is a Go/Rust binary with installable plugins.
# The 'nrf5sdk-tools' plugin provides 'pkg generate' and 'settings generate'
# commands needed for DFU builds. It bundles a compatible Python nrfutil internally.
NRFUTIL_OK=false
if command -v nrfutil &> /dev/null; then
    # Check if nrf5sdk-tools plugin is installed (provides 'pkg' command)
    if nrfutil pkg generate --help &> /dev/null; then
        echo -e "${GREEN}  [OK] nrfutil with nrf5sdk-tools plugin${NC}"
        NRFUTIL_OK=true
    elif nrfutil --help &> /dev/null; then
        echo -e "${YELLOW}  [PARTIAL] nrfutil found but nrf5sdk-tools plugin missing${NC}"
    else
        echo -e "${YELLOW}  [BROKEN] nrfutil found but not working${NC}"
    fi
else
    echo -e "${YELLOW}  [MISSING] nrfutil (needed for DFU builds)${NC}"
fi

if [ "$NRFUTIL_OK" = false ]; then
    if ask_yes_no "Install/configure nrfutil with nrf5sdk-tools?"; then
        # Install nrfutil binary if not present
        if ! command -v nrfutil &> /dev/null; then
            echo -e "${BLUE}  Downloading nrfutil CLI...${NC}"
            mkdir -p "$HOME/.local/bin"
            if curl -fsSL "https://developer.nordicsemi.com/.pc-tools/nrfutil/x64-linux/nrfutil" -o "$HOME/.local/bin/nrfutil"; then
                chmod +x "$HOME/.local/bin/nrfutil"
                export PATH="$HOME/.local/bin:$PATH"
                echo -e "${GREEN}  nrfutil CLI installed${NC}"
            else
                echo -e "${RED}  nrfutil download failed${NC}"
            fi
        fi
        # Install nrf5sdk-tools plugin
        if command -v nrfutil &> /dev/null; then
            echo -e "${BLUE}  Installing nrf5sdk-tools plugin...${NC}"
            nrfutil install nrf5sdk-tools 2>&1
            if nrfutil pkg generate --help &> /dev/null; then
                echo -e "${GREEN}  nrfutil nrf5sdk-tools installed successfully${NC}"
            else
                echo -e "${RED}  nrf5sdk-tools plugin install failed${NC}"
            fi
        fi
    else
        echo -e "${YELLOW}  Skipping nrfutil${NC}"
    fi
    echo ""
fi

# --- Install nrf-command-line-tools (nrfjprog + mergehex) ---
NEED_NRF_CLT=false
if ! command -v nrfjprog &> /dev/null || ! command -v mergehex &> /dev/null; then
    NEED_NRF_CLT=true
fi

if [ "$NEED_NRF_CLT" = true ]; then
    echo -e "${YELLOW}  [MISSING] nrfjprog/mergehex (nRF Command Line Tools)${NC}"
    if ask_yes_no "Download and install nRF Command Line Tools?"; then
        NRF_CLT_VERSION="10.24.2"
        NRF_CLT_ARCHIVE="nrf-command-line-tools-${NRF_CLT_VERSION}_linux-amd64.tar.gz"
        NRF_CLT_URL="https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-24-2/${NRF_CLT_ARCHIVE}"
        NRF_CLT_DIR="/opt/nrf-command-line-tools"

        TMPDIR=$(mktemp -d)
        echo -e "${BLUE}  Downloading nRF Command Line Tools v${NRF_CLT_VERSION}...${NC}"
        if curl -fSL -o "$TMPDIR/$NRF_CLT_ARCHIVE" "$NRF_CLT_URL" 2>/dev/null || \
           wget -q -O "$TMPDIR/$NRF_CLT_ARCHIVE" "$NRF_CLT_URL" 2>/dev/null; then

            echo -e "${BLUE}  Extracting to ${NRF_CLT_DIR}...${NC}"
            sudo mkdir -p "$NRF_CLT_DIR"
            sudo tar xzf "$TMPDIR/$NRF_CLT_ARCHIVE" -C "$NRF_CLT_DIR" --strip-components=1 2>/dev/null || \
            sudo tar xzf "$TMPDIR/$NRF_CLT_ARCHIVE" -C "$NRF_CLT_DIR" 2>/dev/null

            # Try to find the binaries in the extracted tree
            NRFJPROG_BIN=$(find "$NRF_CLT_DIR" -name "nrfjprog" -type f 2>/dev/null | head -1)
            MERGEHEX_BIN=$(find "$NRF_CLT_DIR" -name "mergehex" -type f 2>/dev/null | head -1)

            if [ -n "$NRFJPROG_BIN" ]; then
                NRF_BIN_DIR=$(dirname "$NRFJPROG_BIN")
                sudo ln -sf "$NRFJPROG_BIN" /usr/local/bin/nrfjprog 2>/dev/null
                echo -e "${GREEN}  nrfjprog installed${NC}"
            fi
            if [ -n "$MERGEHEX_BIN" ]; then
                sudo ln -sf "$MERGEHEX_BIN" /usr/local/bin/mergehex 2>/dev/null
                echo -e "${GREEN}  mergehex installed${NC}"
            fi

            # Install bundled SEGGER J-Link if present
            JLINK_DEB=$(find "$NRF_CLT_DIR" -name "JLink_Linux_*.deb" 2>/dev/null | head -1)
            if [ -n "$JLINK_DEB" ]; then
                echo -e "${BLUE}  Installing bundled SEGGER J-Link...${NC}"
                sudo dpkg -i "$JLINK_DEB" 2>/dev/null && \
                    echo -e "${GREEN}  J-Link installed${NC}" || \
                    echo -e "${YELLOW}  J-Link install failed (may need: sudo apt-get -f install)${NC}"
            fi

            if [ -z "$NRFJPROG_BIN" ] && [ -z "$MERGEHEX_BIN" ]; then
                echo -e "${RED}  Could not find binaries in archive${NC}"
                echo -e "${YELLOW}  Download manually from: https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download${NC}"
            fi
        else
            echo -e "${RED}  Download failed${NC}"
            echo -e "${YELLOW}  Download manually from: https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download${NC}"
        fi
        rm -rf "$TMPDIR"
    else
        echo -e "${YELLOW}  Skipping nRF Command Line Tools${NC}"
    fi
    echo ""
else
    echo -e "${GREEN}  [OK] nrfjprog${NC}"
    echo -e "${GREEN}  [OK] mergehex${NC}"
fi

# =============================================================================
# Step 2: Detect tool paths
# =============================================================================
echo ""
echo -e "${BLUE}Step 2: Detecting tool paths${NC}"
echo ""

ARM_TOOLCHAIN_PATH=""
NRFUTIL_PATH=""

# --- Find ARM GCC ---
# Prefer the downloaded toolchain over system version (apt GCC 13.x has issues)
if [ -f "$ARM_GCC_DIR/bin/arm-none-eabi-gcc" ]; then
    ARM_TOOLCHAIN_PATH="$ARM_GCC_DIR/bin"
    echo -e "${GREEN}  arm-none-eabi-gcc: $ARM_TOOLCHAIN_PATH ($($ARM_TOOLCHAIN_PATH/arm-none-eabi-gcc -dumpversion))${NC}"
else
    # Fallback: search common locations
    for path_pattern in "$HOME/tools/gcc-arm-none-eabi-"*/bin /usr/local/gcc-arm-none-eabi-*/bin /opt/gcc-arm-none-eabi-*/bin; do
        for path in $path_pattern; do
            if [ -d "$path" ] && [ -f "$path/arm-none-eabi-gcc" ]; then
                ARM_TOOLCHAIN_PATH="$path"
                echo -e "${GREEN}  arm-none-eabi-gcc: $path ($($path/arm-none-eabi-gcc -dumpversion))${NC}"
                break 2
            fi
        done
    done
    # Last resort: system version
    if [ -z "$ARM_TOOLCHAIN_PATH" ] && command -v arm-none-eabi-gcc &> /dev/null; then
        ARM_TOOLCHAIN_PATH=$(dirname "$(which arm-none-eabi-gcc)")
        echo -e "${YELLOW}  arm-none-eabi-gcc: $ARM_TOOLCHAIN_PATH (system v$(arm-none-eabi-gcc -dumpversion) - may have warnings)${NC}"
    fi
    if [ -z "$ARM_TOOLCHAIN_PATH" ]; then
        echo -e "${RED}  arm-none-eabi-gcc: NOT FOUND${NC}"
        read -p "  Enter ARM GCC bin directory path (or Enter to skip): " manual_arm_path
        [ -n "$manual_arm_path" ] && ARM_TOOLCHAIN_PATH="$manual_arm_path"
    fi
fi

# --- Find nrfutil ---
# Refresh PATH in case pipx just installed it
export PATH="$HOME/.local/bin:$PATH"

if command -v nrfutil &> /dev/null; then
    NRFUTIL_PATH=$(dirname "$(which nrfutil)")
    echo -e "${GREEN}  nrfutil: $NRFUTIL_PATH${NC}"
else
    for path in "$HOME/.nrfutil/bin" "$HOME/.local/bin" /usr/local/bin /usr/bin; do
        if [ -f "$path/nrfutil" ]; then
            NRFUTIL_PATH="$path"
            echo -e "${GREEN}  nrfutil: $path${NC}"
            break
        fi
    done
    if [ -z "$NRFUTIL_PATH" ]; then
        echo -e "${YELLOW}  nrfutil: not found (DFU builds will be limited)${NC}"
    fi
fi

# =============================================================================
# Step 3: Generate build_config.sh
# =============================================================================
echo ""
echo -e "${BLUE}Step 3: Generating build_config.sh${NC}"

cat > "$PROJECT_ROOT/build_config.sh" <<BEOF
#!/bin/bash
# =============================================================================
# Build Configuration File
# =============================================================================
# Generated by setup_environment.sh on $(date)
# You can manually edit this file if paths change.
# Re-run ./scripts/setup_environment.sh to regenerate.
# =============================================================================

# ARM GCC Toolchain path (directory containing arm-none-eabi-gcc)
ARM_TOOLCHAIN_PATH="${ARM_TOOLCHAIN_PATH}"

# nrfutil path (directory containing nrfutil)
NRFUTIL_PATH="${NRFUTIL_PATH}"

# -----------------------------------------------------------------------------
# Build PATH construction (do not edit below)
# -----------------------------------------------------------------------------
BUILD_PATH=""
if [ -n "\$ARM_TOOLCHAIN_PATH" ]; then
    BUILD_PATH="\$ARM_TOOLCHAIN_PATH"
fi
if [ -n "\$NRFUTIL_PATH" ]; then
    if [ -n "\$BUILD_PATH" ]; then
        BUILD_PATH="\$BUILD_PATH:\$NRFUTIL_PATH"
    else
        BUILD_PATH="\$NRFUTIL_PATH"
    fi
fi

if [ -n "\$BUILD_PATH" ]; then
    export PATH="\$BUILD_PATH:\$PATH"
fi

export ARM_TOOLCHAIN_PATH
export NRFUTIL_PATH
BEOF

chmod +x "$PROJECT_ROOT/build_config.sh"
echo -e "${GREEN}  Created: build_config.sh${NC}"

# =============================================================================
# Step 4: Update .vscode/tasks.json
# =============================================================================
echo ""
echo -e "${BLUE}Step 4: Updating .vscode/tasks.json${NC}"

TASKS_PATH=""
[ -n "$ARM_TOOLCHAIN_PATH" ] && TASKS_PATH="$ARM_TOOLCHAIN_PATH"
if [ -n "$NRFUTIL_PATH" ]; then
    [ -n "$TASKS_PATH" ] && TASKS_PATH="$TASKS_PATH:$NRFUTIL_PATH" || TASKS_PATH="$NRFUTIL_PATH"
fi

TASKS_FILE="$PROJECT_ROOT/.vscode/tasks.json"
if [ -f "$TASKS_FILE" ]; then
    cp "$TASKS_FILE" "$TASKS_FILE.backup"
    sed -i "s|\"PATH\": \"[^\"]*\"|\"PATH\": \"$TASKS_PATH:\${env:PATH}\"|" "$TASKS_FILE"
    echo -e "${GREEN}  Updated: .vscode/tasks.json${NC}"
else
    echo -e "${YELLOW}  .vscode/tasks.json not found, skipping${NC}"
fi

# =============================================================================
# Step 4b: Update .vscode/settings.json compiler paths
# =============================================================================
# settings.json is committed and shared, so the three C_Cpp_Runner paths it
# carries are whatever the last committer had. They are the only three
# machine-dependent keys in the file; everything else is portable.
echo ""
echo -e "${BLUE}Step 4b: Updating .vscode/settings.json compiler paths${NC}"

SETTINGS_FILE="$PROJECT_ROOT/.vscode/settings.json"
if [ -f "$SETTINGS_FILE" ] && [ -n "$ARM_TOOLCHAIN_PATH" ]; then
    cp "$SETTINGS_FILE" "$SETTINGS_FILE.backup"
    sed -i \
        -e "s|\"C_Cpp_Runner.cCompilerPath\": \"[^\"]*\"|\"C_Cpp_Runner.cCompilerPath\": \"$ARM_TOOLCHAIN_PATH/arm-none-eabi-gcc\"|" \
        -e "s|\"C_Cpp_Runner.cppCompilerPath\": \"[^\"]*\"|\"C_Cpp_Runner.cppCompilerPath\": \"$ARM_TOOLCHAIN_PATH/arm-none-eabi-g++\"|" \
        -e "s|\"C_Cpp_Runner.debuggerPath\": \"[^\"]*\"|\"C_Cpp_Runner.debuggerPath\": \"$ARM_TOOLCHAIN_PATH/arm-none-eabi-gdb\"|" \
        "$SETTINGS_FILE"
    echo -e "${GREEN}  Updated: .vscode/settings.json -> $ARM_TOOLCHAIN_PATH${NC}"
    echo -e "${YELLOW}  These three lines are yours alone: please do not commit them back.${NC}"
else
    echo -e "${YELLOW}  Skipping (settings.json not found or ARM path not set)${NC}"
fi

# =============================================================================
# Step 5: Update SDK Makefile.posix
# =============================================================================
echo ""
echo -e "${BLUE}Step 5: Updating SDK Makefile.posix${NC}"

MAKEFILE_POSIX="$PROJECT_ROOT/ports/nrf52840/drivers/nRF5_SDK_17.0.2/components/toolchain/gcc/Makefile.posix"
if [ -f "$MAKEFILE_POSIX" ] && [ -n "$ARM_TOOLCHAIN_PATH" ]; then
    cp "$MAKEFILE_POSIX" "$MAKEFILE_POSIX.backup"

    GCC_BINARY="$ARM_TOOLCHAIN_PATH/arm-none-eabi-gcc"
    if [ -f "$GCC_BINARY" ]; then
        GCC_VERSION=$("$GCC_BINARY" -dumpversion)
    else
        GCC_VERSION="unknown"
    fi

    cat > "$MAKEFILE_POSIX" <<MEOF
GNU_INSTALL_ROOT ?= ${ARM_TOOLCHAIN_PATH}/
GNU_VERSION ?= ${GCC_VERSION}
GNU_PREFIX ?= arm-none-eabi
MEOF

    echo -e "${GREEN}  Updated: Makefile.posix (GCC $GCC_VERSION)${NC}"
else
    echo -e "${YELLOW}  Skipping (Makefile.posix not found or ARM path not set)${NC}"
fi

# =============================================================================
# Step 6: Verify configuration
# =============================================================================
echo ""
echo -e "${BLUE}Step 6: Verifying final configuration${NC}"
echo ""

source "$PROJECT_ROOT/build_config.sh"

ALL_OK=true
WARN_COUNT=0

verify_tool() {
    local cmd="$1"
    local required="$2"
    local desc="$3"
    if command -v "$cmd" &> /dev/null; then
        printf "${GREEN}  [OK]   %-22s %s${NC}\n" "$cmd" "$(which $cmd)"
    elif [ "$required" = "required" ]; then
        printf "${RED}  [FAIL] %-22s not found ($desc)${NC}\n" "$cmd"
        ALL_OK=false
    else
        printf "${YELLOW}  [WARN] %-22s not found ($desc)${NC}\n" "$cmd"
        WARN_COUNT=$((WARN_COUNT + 1))
    fi
}

verify_tool "arm-none-eabi-gcc" "required"  "ARM cross-compiler"
verify_tool "make"              "required"  "build tool"
verify_tool "cmake"             "optional"  "needed for unit tests"
verify_tool "ninja"             "optional"  "needed for simulations"
verify_tool "nrfutil"           "optional"  "needed for DFU builds"
verify_tool "mergehex"          "optional"  "needed for bootloader builds"
verify_tool "nrfjprog"          "optional"  "needed for flashing"

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${BLUE}=========================================================================${NC}"
if [ "$ALL_OK" = true ] && [ $WARN_COUNT -eq 0 ]; then
    echo -e "${GREEN}Setup complete! All tools installed and configured.${NC}"
elif [ "$ALL_OK" = true ]; then
    echo -e "${GREEN}Setup complete!${NC} (${WARN_COUNT} optional tools missing)"
else
    echo -e "${RED}Setup incomplete. Required tools are missing.${NC}"
fi
echo ""
echo "Configuration files updated:"
echo "  - build_config.sh"
echo "  - .vscode/tasks.json"
echo "  - .vscode/settings.json  (compiler paths only -- do not commit them back)"
echo "  - ports/.../Makefile.posix"
echo ""
echo "To build:  ./scripts/build_linkitv4_kim.sh  or  ./scripts/build_rspb.sh"
echo "Or use VSCode tasks (Ctrl+Shift+B)"
echo -e "${BLUE}=========================================================================${NC}"
