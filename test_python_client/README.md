# Sliding Window Protocol Test Client

Python-based serial test client for the MCU sliding window protocol implementation.

## Features

- ✅ Full sliding window protocol with 16-bit sequence numbers
- ✅ CRC-16-CCITT error detection
- ✅ Byte stuffing/unstuffing for frame delimiting (0x7E, 0x7D)
- ✅ SACK (Selective Acknowledgment) support
- ✅ Interactive command-line interface
- ✅ Real-time statistics tracking

## Installation

### Requirements

```bash
pip install -r requirements.txt
```

Or install directly:
```bash
pip install pyserial
```

## Usage

### Basic Usage

```bash
python sliding_window_client.py <PORT> [BAUDRATE]
```

Common flags:

- `--json '{...}'` – send a one-off JSON payload supplied inline.
- `--json-file path/to/payload.json` – load JSON from a file.
- `--payload-file path/to/payload.bin` – stream raw payload bytes from disk (no encoding assumptions).
- `--chunk-size 512` – force a specific per-packet payload when splitting sequences.
- `--debug` – dump raw TX/RX frames and parser details.
- `--timeout-ms 500` / `--conn-timeout-ms 10000` – tweak per-byte and overall timeouts (defaults match the UNO sketch).
- `--target generic` – switch to the generic MCU profile (full protocol payload size, no boot delay).
- `--max-payload 256` / `--boot-delay 0.5` – override the active profile’s limits.

### Examples

**Windows:**
```bash
python sliding_window_client.py COM3
python sliding_window_client.py COM5 115200
```

**Linux/Mac:**
```bash
python sliding_window_client.py /dev/ttyUSB0
python sliding_window_client.py /dev/ttyACM0 115200
```

### Target profiles & overrides

> **Note for Arduino UNO testing**: The UNO sketch keeps the Serial buffer footprint small. Limit payloads to **44 bytes or less**; larger frames will overflow the board’s RX buffer and the client will refuse to send them.

```bash
# Arduino UNO profile (44-byte cap, 1.8 s boot delay, flush-after-open)
python sliding_window_client.py --port /dev/ttyUSB0 --target uno

# Generic MCU, keep default (1024-byte) payload cap
python sliding_window_client.py --port /dev/ttyUSB0 --target generic

# Custom board with 256-byte limit and short boot delay
python sliding_window_client.py --port COM7 --target generic --max-payload 256 --boot-delay 0.2

# Force flushing buffers regardless of profile
python sliding_window_client.py --port COM11 --flush-on-connect --max-payload 128
```

### Interactive Commands

Once connected, use these commands:

| Command | Description |
|---------|-------------|
| `<message>` | Send message and wait for echo |
| `send <message>` | Explicit send command |
| `stats` | Display connection statistics |
| `quit` | Exit the client |

### Example Session (text payload)

```
$ python sliding_window_client.py --port COM11 --baud 115200 --json-file payload.json
✓ Connected to COM11 at 115200 baud
→ Sent packet seq=0, len=44, frame_bytes=64, preview='{
  "msg": "Lorem ipsum dolo…'
  Waiting for echo response...
← Received ACK
← Received echo: '{
  "msg": "Lorem ipsum dolor sit amet,"
}'
✓ Echo matches!

============================================================
Statistics:
  Packets sent:     1
  Packets received: 1
  ACKs sent:        1
  NACKs sent:       0
  CRC errors:       0
  Timeouts:         0
============================================================
✓ Disconnected

```

### Raw payload test

To exercise the MCU at its maximum payload size (1,024 bytes by default), use the bundled `payload_1024` binary file:

```
$ python sliding_window_client.py --port COM8 --baud 115200 --payload-file payload_1024 --target generic
✓ Connected to COM8 at 115200 baud
Loaded raw payload: 1024 bytes from payload_1024
→ Sent packet seq=0, len=1024, frame_bytes=1040, preview='\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\x0c\r\x0e\x0f…'
  Waiting for echo response...
← Received echo (1024 bytes)
✓ Echo matches!

============================================================
Statistics:
  Packets sent:     1
  Packets received: 1
  ACKs sent:        1
  NACKs sent:       0
  CRC errors:       0
  Timeouts:         0
============================================================
✓ Disconnected
```

### Sliding-window sequence test

When the payload exceeds the configured per-packet limit (or when you supply `--chunk-size`), the client automatically splits data across multiple frames and reassembles the echoed response. Example with two 512-byte chunks:

```
$ python sliding_window_client.py --port COM8 --baud 115200 --payload-file payload_1024 --chunk-size 512 --target generic
✓ Connected to COM8 at 115200 baud
Loaded raw payload: 1024 bytes from payload_1024
→ Sending sequence of 2 packets (1024 bytes total, chunk=512)
→ Sent packet seq=0/2, len=512, frame_bytes=528, preview='\x00\x01\x02\x03\x04\x05\x06\x07…'
→ Sent packet seq=1/2, len=512, frame_bytes=528, preview='\x80\x81\x82\x83\x84\x85\x86\x87…'
  Waiting for echo sequence...
→ Sent ACK seq_base=0, bitmap=0x01
→ Sent ACK seq_base=1, bitmap=0x01
← Received sequence (1024 bytes across 2 packets)
✓ Echo matches!

============================================================
Statistics:
  Packets sent:     2
  Packets received: 2
  ACKs sent:        2
  NACKs sent:       0
  CRC errors:       0
  Timeouts:         0
============================================================
✓ Disconnected
```
### Debug session
```
$ python sliding_window_client.py --port COM11 --baud 115200 --json-file payload.json --debug
✓ Connected to COM8 at 115200 baud
[DBG RAW TX] 7e01000001002c0000002c00027b0a20202020226d7367223a20224c6f72656d20697073756d20646f6c6f722073697420616d65742c220a7d5dcdab219f7e
→ Sent packet seq=0, len=44, frame_bytes=64, preview='{
    "msg": "Lorem ipsum dolo…'
  Waiting for echo response...
[DBG RAW RX] 0x7E
[DBG RAW RX] 0x01
[DBG RAW RX] 0x01
[DBG RAW RX] 0x00
[DBG RAW RX] 0x01
[DBG RAW RX] 0x00
[DBG RAW RX] 0x2C
...
[DBG RAW RX] 0x7E
[DBG RAW FRAME] 01010001002c0000002c00017b0a20202020226d7367223a20224c6f72656d20697073756d20646f6c6f722073697420616d65742c220a7dcdabc346
[DBG] Data frame: seq=1, len=44, enc=1, token=0xABCD
← Received echo: '{
    "msg": "Lorem ipsum dolor sit amet,"
}'
[DBG RAW TX] 7e060001017e
→ Sent ACK seq_base=1, bitmap=0x01
✓ Echo matches!

============================================================
Statistics:
  Packets sent:     1
  Packets received: 1
  ACKs sent:        1
  NACKs sent:       0
  CRC errors:       0
  Timeouts:         0
============================================================
✓ Disconnected
```

## How It Works

### Communication Flow

```
PC Client                          MCU
    |                               |
    |---[DATA: "Hello"]------------>|  (framed packet with CRC)
    |                               |
    |                               |  (validate CRC, decode payload)
    |                               |
    |<--[DATA: "Hello"]-------------|  (echo back framed)
    |                               |
    |---[ACK]---------------------->|  (acknowledge receipt)
    |                               |
```

### Packet Structure

```
┌──────────────┬────────────────────┬──────────────┐
│  FRAME_BYTE  │   ESCAPED DATA     │  FRAME_BYTE  │
│    (0x7E)    │                    │    (0x7E)    │
└──────────────┴────────────────────┴──────────────┘
```

**Escaped Data Content:**
```c
flags         (1 byte)  // FLAG_LAST_PACKET = 0x01
seq           (2 bytes) // Sequence number
seq_length    (2 bytes) // Total packets in sequence
seq_size      (4 bytes) // Total data size
data_length   (2 bytes) // Length of data in this packet
encoding_type (1 byte)  // ENCODING_ASCII = 0x01
data          (variable)// Actual payload
token         (2 bytes) // Security token (0xABCD)
crc           (2 bytes) // CRC-16-CCITT checksum
```

### Byte Stuffing Rules

Special characters are escaped to avoid confusion with frame delimiters:

| Original | Escaped Sequence |
|----------|-----------------|
| `0x7E` (FRAME_BYTE) | `0x7D 0x5E` |
| `0x7D` (ESCAPE_CHAR) | `0x7D 0x5D` |

### CRC-16 Calculation

- **Algorithm:** CRC-16-CCITT
- **Polynomial:** 0x1021
- **Initial Value:** 0xFFFF
- **Computed Over:** All packet fields except CRC itself

## Protocol Details

### Encoding Types

| Value | Type | Description |
|-------|------|-------------|
| 0x00 | BINARY | Raw binary data |
| 0x01 | ASCII | ASCII text (default) |
| 0x02 | UTF8 | UTF-8 encoded text |
| 0x03 | JSON | JSON formatted data |
| 0x04 | CBOR | CBOR encoded data |
| 0x05 | BASE64 | Base64 encoded |
| 0x06 | HEX | Hexadecimal string |

### Control Frames

| Byte | Type | Description |
|------|------|-------------|
| 0x06 | ACK | Acknowledgment |
| 0x15 | NACK | Negative acknowledgment |
| 0x55 | RESET | Reset connection |

## Troubleshooting

### Port Access Issues (Linux)

Add your user to the `dialout` group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

Or temporarily:
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Port Busy (Windows)

- Close any other serial terminal (PuTTY, Arduino IDE, etc.)
- Check Device Manager for COM port number
- Ensure MCU is powered and connected

### No Response from MCU

1. **Verify port:** Check the correct COM port in Device Manager (Windows) or `ls /dev/tty*` (Linux)
2. **Check baudrate:** Default is 115200, must match MCU configuration
3. **Ensure MCU is running:** Verify the firmware is flashed and running
4. **Check connections:** TX→RX, RX→TX, GND must be connected
5. **Check UART settings:** 115200 baud, 8N1 (8 data bits, no parity, 1 stop bit)

### CRC Errors

- Check for electrical noise on UART lines
- Verify baudrate matches exactly
- Ensure proper grounding between devices
- Try lower baudrate (e.g., 9600)

### Module Not Found

```bash
pip install pyserial
```

Or with requirements file:
```bash
pip install -r requirements.txt
```

## Testing Without MCU

For protocol development/testing, use a serial loopback:

**Physical Loopback:**
- Connect TX to RX on the same device
- Add a resistor (220Ω) for safety

**Software Loopback (Linux):**
```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# Use the two created virtual ports
```

**Windows:**
- Use [com0com](https://sourceforge.net/projects/com0com/) virtual COM port pairs

## Advanced Usage

### Scripted Testing

```python
from sliding_window_client import SlidingWindowClient

client = SlidingWindowClient('COM3', 115200)
if client.connect():
    # Send test messages
    client.send_message("Test 1")
    client.send_message("Test 2")
    client.print_stats()
    client.disconnect()
```

### Batch Testing

```bash
# Create test_messages.txt with messages (one per line)
# Then pipe to the client
cat test_messages.txt | python sliding_window_client.py COM3
```

## Contributing

When modifying the protocol:
1. Update both MCU (`sliding_window_protocol_16bit.c`) and Python client
2. Maintain CRC-16 compatibility
3. Test byte stuffing edge cases (0x7E, 0x7D in payload)
4. Verify sequence number wrapping at 65536

## License

Matches the MCU implementation license.
