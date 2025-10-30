# C++ WT901C485 reader

Minimal C++ Modbus RTU reader using termios and CMake.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Or use the helper script:

```bash
chmod +x build.sh
# Build Release
./build.sh
# Clean only
./build.sh --clean
# Rebuild (clean-first) with N jobs
./build.sh --rebuild -j 4
# Debug build
./build.sh --debug
# Build and run
./build.sh --run "--port /dev/ttyCH341USB0 --baud 115200 --addr 0x50"
```

## Run

```bash
./build/wt901_reader --port /dev/ttyCH341USB0 --baud 115200 --addr 0x50
```

Press Ctrl-C to stop. Ensure your user has access to the serial device.


