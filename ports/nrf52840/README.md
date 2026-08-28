# LinkIt V4 Application

## How to build

The LinkIt V4 application project uses CMake and the arm-none-eabi-gcc compiler.

See the [Build section of the root README](../../README.md#2-build) for full build instructions, or use the build scripts in [`../../scripts/`](../../scripts/).

### Quick build

```bash
# From project root
./scripts/build_linkitv4_kim.sh          # LinkIt V4 KIM
./scripts/build_linkitv4_smd.sh  # LinkIt V4 SMD
./scripts/build_linkitv4_lora.sh # LinkIt V4 LoRa
./scripts/build_rspb.sh          # RSPB
```

### Manual CMake build

```bash
mkdir -p build/LINKIT && cd build/LINKIT
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
  -DCMAKE_BUILD_TYPE=Debug -DDEBUG_LEVEL=4 -DBOARD=LINKIT ../..
make -j$(nproc)
```

## How to flash the firmware in DFU mode

With the device connected via USB:
```
make dfu_app
```
