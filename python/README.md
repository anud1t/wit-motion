# Python WT901C485 reader

Minimal Modbus RTU reader for WT901C485 using `pyserial`.

## Install

```bash
python3 -m pip install -r requirements.txt
```

## Run

```bash
python3 wt901_reader.py --port /dev/ttyCH341USB0 --baud 115200 --addr 0x50
```

Press Ctrl-C to stop. Ensure your user has access to the serial device (e.g., group `dialout`).


