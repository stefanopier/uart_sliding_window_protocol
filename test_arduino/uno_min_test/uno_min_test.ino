#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 128
#endif
#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 128
#endif

#include <Arduino.h>

// Arduino UNO test harness for the sliding window protocol
// - Uses Serial as UART transport
// - Implements the platform hooks expected by the protocol
// - Includes the C implementation directly with C linkage

#define BAUD_RATE 115200

// Provide C linkage for hook functions so the C protocol can link to them
extern "C" {
  // Forward declare blink so swp_debug_pulse can call it (defined later)
  void blink(uint8_t times, uint16_t on_ms, uint16_t off_ms);
  bool uart_send_byte(uint8_t byte) {
    Serial.write(byte);
    return true;
  }

  void uart_flush(void) {
    Serial.flush();
  }

  uint8_t uart_receive_byte(void) {
    while (Serial.available() <= 0) {
      // tight wait; protocol has timeouts upstream
    }
    return (uint8_t)Serial.read();
  }

  bool uart_RX_available(void) {
    return Serial.available() > 0;
  }

  // Wrapper to avoid C/C++ name mangling conflicts with Arduino's millis()
  uint32_t swp_millis(void) {
    return ::millis();
  }

  // debug pulse called from C protocol implementation
  void swp_debug_pulse(uint8_t code) {
    // code 1: ACK sent -> short single blink
    // code 2: full message assembled -> two short blinks
    // code 3: packet sent -> three short blinks
    const uint16_t fast_on = 5;
    const uint16_t fast_off = 5;

    if (code == 1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(40);
        digitalWrite(LED_BUILTIN, LOW);
      } else if (code == 2) {
        blink(2, 30, 30);
      } else if (code == 3) {
        blink(3, 25, 25);
      } else if (code == 4) {
        // Code 4: reached echo transmit path (longer visual pattern)
        blink(4, 60, 40);
      } else if (code >= 5 && code <= 9) {
        // Fast strobes so we do not hold up UART parsing during diagnostics
        blink(code, fast_on, fast_off);
      } else if (code == 10) {
        blink(4, 150, 80); // long flashes -> sequence validation failure
      } else if (code == 11) {
        blink(1, 200, 120); // very long single pulse -> frame-start timeout
      } else if (code == 12) {
        blink(2, 160, 100); // two long pulses -> byte timeout mid-frame
      }
  }
}

// Build the protocol as C inside this C++ sketch.
// We map millis -> swp_millis so the C file calls the wrapper above.
extern "C" {
  // Use small-footprint defaults from the header (WINDOW_SIZE=1, MAX_DATA_SIZE=128)
  #define millis swp_millis
  #include "sliding_window_protocol_16bit.h"
  // Do NOT #include the .c file here — Arduino build system compiles the .c separately.
}

// Test buffers
static uint8_t rx_buffer[128];
static bool debug_probe_sent = false;

static void send_debug_probe(void) {
  // Send a single SACK-style frame (ACK with zero bitmap) so the host can confirm RX path
  const uint8_t probe_frame[] = { FRAME_BYTE, ACK, 0x00, 0x00, 0x00, FRAME_BYTE };
  Serial.write(probe_frame, sizeof(probe_frame));
  Serial.flush();
  debug_probe_sent = true;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(BAUD_RATE);
  // Give host time to open the port
  delay(500);

  // Send a probe frame once so the host can verify UART RX from the MCU side
  send_debug_probe();

  init_sliding_window_protocol();
}

void blink(uint8_t times, uint16_t on_ms = 50, uint16_t off_ms = 50) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(on_ms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(off_ms);
  }
}

void loop() {
  uint16_t out_len = 0;
  uint8_t enc = ENCODING_BINARY;

  // Blocking receive of one complete logical message
  reliable_receive_buffered_with_encoding(rx_buffer, &out_len, &enc);

  if (out_len > 0) {
    // Visual feedback
    blink(2);

    // Debug: indicate we're about to send the echo back
    swp_debug_pulse(4);

    // Echo the payload back using the same encoding to validate TX path
    reliable_send_buffered_with_encoding(rx_buffer, out_len, enc);

    // Clear for next
    out_len = 0;
  }
}
