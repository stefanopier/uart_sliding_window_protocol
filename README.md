# uart_sliding_window_protocol

This repository contains an implementation of a lightweight UART-based sliding-window protocol with 16-bit sequence numbers, SACK (selective acknowledgements), CRC-16 validation, a shared security token, and several encoding types.

This README documents the wire format, framing rules, primary constants, control frames, and public API used in the implementation. There is also a small-footprint variant under `test_arduino/uno_min_test` which shows how to tune the protocol for constrained MCUs such as an Arduino UNO.

---

## Protocol overview ✅

- 16-bit sequence numbers (wrap-around supported)
- Configurable sliding window (via `WINDOW_SIZE` macro)
- Break payloads into multiple packets using `seq_length` and `seq_size`
- SACK support — receiver returns a bitmap of received packets in the current window
- CRC-16 (x^16 + x^12 + x^5 + 1) using polynomial `0x1021` (CRC-CCITT) on header+data+token
- Shared token to prevent cross-talk or accidental frames from unrelated hosts

---

## Wire format (per-frame) 🔌

Frame boundaries are indicated with a frame marker and susceptible bytes are escaped.

Wire format (logical):

1. FRAME_BYTE (0x7E)
2. header (12 bytes):
	- flags (1 byte)
	- seq (2 bytes, little-endian)
	- seq_length (2 bytes, little-endian) — total packets in full sequence
	- seq_size (4 bytes, little-endian) — total data length across all packets
	- data_length (2 bytes, little-endian)
	- encoding_type (1 byte)
3. data (data_length bytes)
4. token (2 bytes, little-endian)
5. crc (2 bytes, little-endian) — CRC of header + data + token
6. FRAME_BYTE (0x7E)

Byte-stuffing characters:
- `FRAME_BYTE = 0x7E`
- `ESCAPE_CHAR = 0x7D`
- `STUFF_BYTE = 0x20` (XOR mask) — the receiver XORs escaped bytes with `STUFF_BYTE` to recover the original.

Control frames are single control bytes inside a frame (ACK, NACK, RESET) and may be followed by control data and framing. Some error paths may also emit a raw `RESET_FRAME` byte (0x55) without framing as an emergency reset signal; peers that support this should treat a lone 0x55 as an out-of-band reset.

---

## Frame sizing (implementation guidance) 📐

To help third-party implementations allocate buffers safely, the reference header defines these sizing macros:

- `SWP_FRAME_HEADER_LEN` = 12 bytes
- `SWP_FRAME_TRAILER_LEN` = 4 bytes (token + CRC)
- `SWP_MAX_FRAME_UNSTUFFED` = header + payload + trailer
- `SWP_MAX_FRAME_STUFFED` = worst-case byte-stuffed size (+2 for delimiters)

These values are derived directly from the wire format above and match the MCU implementation.

---

## Constants and flags 🔢

Important constants in the header:

- `FRAME_BYTE` — 0x7E
- `ESCAPE_CHAR` — 0x7D
- `STUFF_BYTE` — 0x20
- `ACK` — 0x06
- `NACK` — 0x15
- `RESET_FRAME` — 0x55
- `MAX_DATA_SIZE` — configured by `SWP_MAX_DATA_SIZE`
- `WINDOW_SIZE` — configured by `SWP_WINDOW_SIZE`

Note: The `SWP_*` prefix denotes Sliding Window Protocol-related defines.

Flags used in frame `flags` byte (some are project-specific):

`FLAG_LAST_FRAME` is a marker bit (0x01) that may be OR’d into the base flag to indicate the final fragment of a sequence. All other flag values below are base types; OR with `FLAG_LAST_FRAME` when needed.

### Stardome flags (base values)

| Flag | Value (hex) | Notes |
|------|-------------|-------|
| `FLAG_SIGN` | 0x02 | SIGN request (CBOR payload) |
| `FLAG_STARDOME_ATTESTATION` | 0x04 | Attestation request |
| `FLAG_STARDOME_PROOF` | 0x08 | Proof request |
| `FLAG_STARDOME_STATUS` | 0x10 | Status request |
| `FLAG_STARDOME_DATA` | 0x12 | Data request |
| `FLAG_STARDOME_TREE` | 0x20 | Merkle tree request |
| `FLAG_STARDOME_HOST_ID` | 0x40 | Host ID request |
| `FLAG_STARDOME_LOWMODE` | 0x50 | Low-power mode |
| `FLAG_STARDOME_HIGHMODE` | 0x70 | High-power mode |
| `FLAG_STARDOME_OFF` | 0x80 | Power off |
| `FLAG_STARDOME_STATUS_DATA` | 0x90 | Status data response |
| `FLAG_BOARD_STATUS` | 0xA0 | Board status request |
| `FLAG_BOARD_STATUS_DATA` | 0xB0 | Board status response |
| `FLAG_STARDOME_PROOF_DATA` | 0xB2 | Proof data response |

### Error response flags

Error responses use a base error flag and a 1-byte payload error code. The error flag is OR’d with `FLAG_LAST_FRAME` when sent. In some internal error paths (e.g., SIGN handler failure), the implementation may send a generic 1-byte error payload with only `FLAG_LAST_FRAME` set (no specific error flag).

| Error flag | Value (hex) | Applies to |
|-----------|-------------|------------|
| `FLAG_SIGN_ERROR` | 0xC2 | `FLAG_SIGN` |
| `FLAG_STARDOME_PROOF_ERROR` | 0xC4 | `FLAG_STARDOME_PROOF` |
| `FLAG_STARDOME_DATA_ERROR` | 0xC6 | `FLAG_STARDOME_DATA` |
| `FLAG_STARDOME_STATUS_ERROR` | 0xC8 | `FLAG_STARDOME_STATUS` |
| `FLAG_BOARD_STATUS_ERROR` | 0xCA | `FLAG_BOARD_STATUS` |
| `FLAG_STARDOME_HOST_ID_ERROR` | 0xCC | `FLAG_STARDOME_HOST_ID` |
| `FLAG_STARDOME_OFF_ERROR` | 0xCE | `FLAG_STARDOME_OFF` |
| `FLAG_STARDOME_LOWMODE_ERROR` | 0xD8 | `FLAG_STARDOME_LOWMODE` |
| `FLAG_STARDOME_HIGHMODE_ERROR` | 0xDA | `FLAG_STARDOME_HIGHMODE` |

Flags that do not have a dedicated error flag in the MCU implementation should be treated as NACK-only errors by default.

---

## Control frames and SACK 📨

- ACK: ack frame begins with `ACK` followed by 16-bit base seq (big-endian) and a bitmap that represents reception of subsequent packets within the `WINDOW_SIZE`. The bitmap is used for SACK.
- NACK: simple NACK frame (no body in this implementation); senders can trigger retransmit policy.
- RESET: a control frame instructing both sides to reset sequence numbers and state.

SACK bitmap example (WINDOW_SIZE=8): bitmap bit 0 corresponds to `base_seq`, bit1 to `base_seq+1`, …

---

## CRC & Token 🛡️

- The CRC is computed over header + data + token using CRC-16-CCITT (poly 0x1021) and stored in the trailing 2 bytes.
- `SHARED_TOKEN` (default `0xABCD`) must match on both peers; otherwise, the frame is discarded.

---

## Public API — main functions (C)

- `init_sliding_window_protocol()` — resets internal state.
- `reliable_send_buffered(const uint8_t *data, uint16_t total_len)` — block until all payload sent and acked.
- `reliable_send_buffered_with_encoding(...)` — like above with an encoding type param.
- `reliable_receive_buffered(uint8_t *output, volatile uint16_t *output_len, uint16_t output_max)` — accumulate frames into `output` until `FLAG_LAST_FRAME` is seen. `output_max` bounds-checks the buffer to prevent overflow.
- `receive_decoded_frame(...)` — non-blocking helper to decode a single incoming frame and return its payload.
- `get_connection_statistics(ConnectionState *stats)` — retrieve counters used for debugging/telemetry.
- `reset_connection_statistics()` — zero out statistics.
- `reset_sliding_window_receiver()` — clear only receiver state (expected frame index and reorder buffer). Use between independent receive transactions.
- `reset_sliding_window_after_tx(uint32_t timeout_ms)` — drain the TX window (wait for ACKs), then full-reset transport state. Returns `true` if all frames were acked before timeout.
- `send_error_response(uint8_t error_flag, uint8_t error_code)` — send a single-byte error payload with flags = `error_flag | FLAG_LAST_FRAME`.

### Transaction lifecycle

For multi-request servers or devices handling consecutive sequences:

1. Call `init_sliding_window_protocol()` once at startup.
2. Receive a request via `reliable_receive_buffered()`.
3. Send a response via `reliable_send_buffered()` or `sliding_window_send_fragmented()`.
4. Call `reset_sliding_window_after_tx(CONNECTION_TIMEOUT_MS)` to drain ACKs and prepare for the next transaction.
5. Call `reset_sliding_window_receiver()` if you need to accept a new inbound sequence starting at idx=0.
6. Go to step 2.

Without explicit resets, the receiver will auto-reset when `frame_index=0` arrives while expecting a different index. However, explicit resets are recommended for clarity and to ensure sender state is also cleaned up.

Note: The receiver and sender expect the platform to implement these hooks in user code:

- `bool uart_send_byte(uint8_t byte)` — push a single byte out the UART.
- `uint8_t uart_receive_byte(void)` — read a byte from RX.
- `bool uart_RX_available(void)` — non-blocking; returns whether the serial buffer has >=1 byte ready.
- `uint32_t millis(void)` — millisecond clock for timeout handling.
- `void uart_flush_tx(void)` — (optional) block until all queued TX bytes have been physically transmitted. Default weak implementation is a no-op.

---

## Data encoding types table 🗂️

| ID (hex) | Name | Description |
|---------:|------|-------------|
| 0x00 | ENCODING_BINARY | Raw binary data |
| 0x01 | ENCODING_ASCII | ASCII text |
| 0x02 | ENCODING_UTF8 | UTF8 text |
| 0x03 | ENCODING_JSON | JSON |
| 0x04 | ENCODING_CBOR | CBOR |
| 0x05 | ENCODING_BASE64 | Reserved for future use (not accepted by default implementation) |
| 0x06 | ENCODING_HEX | Hex string |
| 0x07 | ENCODING_TLV_BINARY | TLV binary |
| 0x08 | ENCODING_TLV_ASCII | TLV ASCII |
| 0x09 | ENCODING_TLV_UTF8 | TLV UTF-8 |

Use `get_encoding_type_name()` to return a readable name for a given encoding type.

---

## Arduino UNO small-footprint notes 🧩

The sample in `test_arduino/uno_min_test` shows how to set `SWP_WINDOW_SIZE`, `SWP_MAX_DATA_SIZE`, and timeouts for memory-constrained platforms.

Examples from the UNO test header:

```c
// For UNO: reduce memory
#ifndef SWP_WINDOW_SIZE
#define SWP_WINDOW_SIZE 1
#endif
#ifndef SWP_MAX_DATA_SIZE
#define SWP_MAX_DATA_SIZE 44
#endif
```

Additionally the UNO test uses small debug pulses via `swp_debug_pulse(code)` and flush via `uart_flush()` to make behavior visible during development over USB-CDC. Keep these if you need runtime instrumentation but remove them in production builds for minimal footprint.

---

## Platform extensions (wrapper pattern) 🔧

The protocol implementation keeps all internal state and helper functions `static` (file-private) by default. For platforms that need to extend the protocol — e.g., interleaving RX drain during TX, adding hooks on reset or NACK, or implementing streamed/incremental receive — a compile-time opt-in flag exposes the necessary internals.

### Enabling

Add `-DSWP_PLATFORM_EXTENSIONS` (or `#define SWP_PLATFORM_EXTENSIONS` before including the `.c`) to your build. This:

1. Makes internal state variables non-static (`recv_window`, `send_window`, `conn_state`, sequence indices), accessible via `extern` declarations in `swp_internal.h`.
2. Makes ~10 internal helper functions non-static (`send_escaped_byte`, `receive_any_frame_timed`, `store_frame`, `send_sack_ack`, `send_nack`, `handle_sack_ack`, `check_timeouts_and_retransmit`, `handle_nack_retransmit`, `handle_reset_frame`, `compute_crc16`), so a wrapper translation unit can call them.
3. Makes `send_frame_struct()` a weak symbol (`__attribute__((weak))`), so a wrapper can provide a strong override (e.g., to add per-byte TX pump for DMA coexistence).
4. Activates two hook call-sites with weak no-op defaults:
   - `swp_hook_on_reset()` — called at the end of `handle_reset_frame()`. Override to clear platform-specific state (e.g., QSPI drain buffers).
   - `swp_hook_post_nack()` — called after `send_nack()` sends the NACK frame. Override for post-NACK RX drain.
5. Moves the internal `FrameType` enum to `swp_internal.h` (guarded to avoid redefinition).

**Without the flag, the module behaves identically to previous versions** — all symbols remain `static`, no hooks are called, no weak attributes.

### Required files for a wrapper project

```
your_project/
  sliding_window_protocol_16bit.h   ← public API (unmodified copy)
  sliding_window_protocol_16bit.c   ← protocol implementation (unmodified copy)
  swp_internal.h                    ← internal declarations header
  swp_yourplatform_wrapper.h        ← platform-specific public API
  swp_yourplatform_wrapper.c        ← platform-specific extensions
```

### swp_internal.h

This header (shipped alongside the protocol files) provides:
- The `FrameType` enum (used by both the `.c` and the wrapper)
- `extern` declarations for the 6 internal state variables
- Prototypes for the exposed helper functions
- A `static inline` copy of `crc16_ccitt_update()` (each TU gets its own inlined copy)
- Prototypes for the two weak hooks

Include it from your wrapper `.c` only — never from application code.

### Writing a platform wrapper

A wrapper `.c` file links alongside the unmodified protocol `.c`. It can:

- **Override `send_frame_struct()`** — provide a strong definition that replaces the weak default. Example: interleave per-byte RX pump calls during TX to prevent DMA overflow on MCUs with shared UART/DMA channels.
- **Override `swp_hook_on_reset()`** — provide a strong definition to clear domain-specific state on protocol reset.
- **Override `swp_hook_post_nack()`** — provide a strong definition to drain RX after sending NACK.
- **Add new receive modes** — e.g., `reliable_receive_streamed()` that uses a frame-sink callback instead of accumulating into a flat buffer, enabling payloads larger than RAM.
- **Add cooperative polling** — e.g., `sliding_window_protocol_tasks()` for non-blocking main-loop integration.

Example wrapper skeleton:

```c
// swp_myplatform_wrapper.c
#include "swp_myplatform_wrapper.h"
#include "swp_internal.h"

// Strong override: custom send with TX pump
void send_frame_struct(Frame *frame) {
    // ... your platform-specific frame sending logic
}

// Strong override: clear platform state on reset
void swp_hook_on_reset(void) {
    my_platform_reset_buffers();
}

// Strong override: drain RX after NACK
void swp_hook_post_nack(void) {
    my_platform_drain_rx();
}
```

### Build integration

For MPLAB X / XC32 projects, add `SWP_PLATFORM_EXTENSIONS=1` to the preprocessor macros in your project properties (or `configurations.xml`). For GCC/Make projects, add `-DSWP_PLATFORM_EXTENSIONS` to `CFLAGS`.

Both the protocol `.c` and the wrapper `.c` must be compiled and linked. The linker resolves strong overrides over weak defaults automatically.

---

## Testing locally 🧪

You can use the `test_python_client` folder to exercise the protocol from a PC host. The Python test client will send payloads with different sizes and encodings and verify reception and retransmissions.

Basic run (with Python 3 installed):

```powershell
cd test_python_client
python -m pip install -r requirements.txt
python sliding_window_client.py --device COMx --payload payload.json
```

Replace `COMx` with your device name. The tests will attempt to connect and verify SACK, CRC, and retransmissions.

---

## Contributing & Notes

- The header `sliding_window_protocol_16bit.h` includes extensive documentation, enums, macros and public prototypes; consult that file for precise constants.
- If you modify wire format or sequence numbering, update `test_python_client` tests accordingly.
- For constrained platforms: use the UNO test as a reference and ensure that `MAX_DATA_SIZE` and `WINDOW_SIZE` are safe for your memory budget.

---

## Acknowledgement / Origin

This project was originally forked and inspired by the MIN project — see https://github.com/min-protocol/min for the upstream implementation and design ideas. The MIN repository provided the starting point for the protocol and several reliability and small-footprint patterns; this repository adapts and extends those concepts and includes Arduino-specific adjustments (see `test_arduino/uno_min_test`).

---

