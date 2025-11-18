# Arduino UNO Sliding Window Protocol Test

This sketch builds and runs the protocol on an Arduino UNO using Serial as the transport. It echoes any received payload back to the host using the same encoding. ACK/SACK/CRC-16, byte-stuffing, and token checks are exercised end-to-end.

## Wiring
- Use the board's native USB. No extra wiring needed.

## Build & Upload (Arduino IDE)
- Board: `Arduino UNO`
- Port: your board's COM port
- Baud: `115200`
- Open the folder `test_arduino/uno_min_test` as a sketch.
- Upload.

Note: The sketch includes a local copy of the protocol sources to satisfy Arduino IDE include rules.
For UNO, a small-footprint configuration is used (WINDOW_SIZE=1, MAX_DATA_SIZE=44) to fit 2 KB SRAM and stay within the 64-byte Serial buffer.

## Run the Python client
From your PC:

```bash
# In repo root
cd test_python_client
pip install -r requirements.txt

# Keep payloads <= 44 bytes for UNO; larger frames overflow the hardware Serial buffer.
# Example small JSON:
python sliding_window_client.py --port COMx --baud 115200 \
	--encoding json --json '{"msg":"hello from host"}'
```

Replace `COMx` with your serial port (e.g., `COM5` on Windows or `/dev/ttyACM0` on Linux).

## Expected behavior
- The UNO receives the message, sends SACK ACKs during receipt, validates CRC-16 and token, then blinks the LED twice upon full message assembly.
- It then echoes the payload back to the host using the same encoding.

### LED diagnostics
The sketch flashes the onboard LED to visualize protocol events without a serial console:

- **1 short blink** – ACK frame sent to the host.
- **2 short blinks** – complete payload reassembled.
- **3 short blinks** – outbound data frame transmitted by the UNO.
- **4 medium blinks** – echo transmit path reached in the loop.
- **5 very fast blinks** – inbound data frame fully validated.
- **11 long blink** – timed out waiting for frame start after seeing activity.
- **12 two long blinks** – timed out between bytes while reading a frame.

Other rapid multi-blink patterns (6–10) call out validation failures (length, CRC, token, sequence). If these appear, check the matching debug code in `swp_debug_pulse` inside the sketch.

## Troubleshooting
- If the serial monitor is open, close it before running the Python client (it will interfere with frames).
- If timeouts occur, ensure the selected COM port and baud match.
- If build errors mention `millis`, ensure you didn't move files: the sketch maps `millis` to a wrapper to avoid C/C++ linkage issues.
- If you need larger payloads or a wider window, use a larger MCU (e.g., Mega/Teensy) or adjust `SWP_MAX_DATA_SIZE`/`SWP_WINDOW_SIZE` and verify SRAM usage.
