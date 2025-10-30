# wit-motion IMU examples

This repo contains minimal examples for reading WITMOTION WT901C485 over RS-485/Modbus RTU.

- `python/`: Python reader using `pyserial`, no external dependencies beyond pip.
- `cpp/`: C++ example with CMake, using POSIX termios.

## Quick start

- Python
  - Install: `python3 -m pip install -r python/requirements.txt`
  - Run: `python3 python/wt901_reader.py --port /dev/ttyCH341USB0 --baud 115200 --addr 0x50`

- C++
  - Build:
    - `cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release`
    - `cmake --build cpp/build -j`
  - Run: `./cpp/build/wt901_reader --port /dev/ttyCH341USB0 --baud 115200 --addr 0x50`

  - Or use helper script:
    - `cd cpp && chmod +x build.sh`
    - Build release: `./build.sh`
    - Clean only: `./build.sh --clean`
    - Rebuild (clean-first) with 4 jobs: `./build.sh --rebuild -j 4`
    - Debug build: `./build.sh --debug`
    - Build and run: `./build.sh --run "--port /dev/ttyCH341USB0 --baud 115200 --addr 0x50"`

Notes:
- Use a USB-RS485 adapter (auto-direction) for simplest setup.
- Ensure your user has access to the serial device (e.g., add to `dialout`).