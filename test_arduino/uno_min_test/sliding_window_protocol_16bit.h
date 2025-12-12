/* Copied from project root and adapted with small-footprint overrides for Arduino UNO */
#ifndef SLIDING_WINDOW_PROTOCOL_16BIT_H
#define SLIDING_WINDOW_PROTOCOL_16BIT_H

#include <stdint.h>
#include <stdbool.h>

/* Small-footprint overrides for constrained MCUs (e.g., Arduino UNO). 
 * You can override these by defining SWP_WINDOW_SIZE, SWP_MAX_DATA_SIZE, etc., before including this header.
 */
#ifndef SWP_WINDOW_SIZE
#define SWP_WINDOW_SIZE 1
#endif
#ifndef SWP_MAX_DATA_SIZE
#define SWP_MAX_DATA_SIZE 44
#endif
#ifndef SWP_TIMEOUT_MS
#define SWP_TIMEOUT_MS 150
#endif
#ifndef SWP_CONN_TIMEOUT_MS
#define SWP_CONN_TIMEOUT_MS 3000
#endif

/* 
 * sliding_window_protocol_16bit.h
 *
 * Declarations for an enhanced sliding window protocol with 16-bit sequence numbers,
 * SACK, CRC-16, security token, encoding types, and MIN-inspired reliability.
 */

// #############################################################################
// ## Protocol Constants
// #############################################################################

#define WINDOW_SIZE           SWP_WINDOW_SIZE
#define MAX_SEQ_NUM           65536
#define MAX_DATA_SIZE         SWP_MAX_DATA_SIZE
#define TIMEOUT_MS            SWP_TIMEOUT_MS
#define MAX_RETRANSMIT_COUNT  5
#define CONNECTION_TIMEOUT_MS SWP_CONN_TIMEOUT_MS
#define ACK_TIMEOUT_MS        1000
#define BACKOFF_MULTIPLIER    2
#define ACK                   0x06
#define NACK                  0x15
#define RESET_FRAME           0x55
#define FLAG_LAST_PACKET      0x01
#define FLAG_SIGN             0x02                  // where the last bit is reserved for last packet = 0x03
#define FLAG_STARDOME_SIGNATURE 0x04    // where the last bit is reserved for last packet = 0x05
#define FLAG_STARDOME_TREE    0x26         // where the last bit is reserved for last packet = 0x27
#define FLAG_STARDOME         0x08              // where the last bit is reserved for last packet = 0x09
#define FLAG_SN               0x0A                    // where the last bit is reserved for last packet = 0x0B
#define FLAG_STARDOME_SN      0x0C           // where the last bit is reserved for last packet = 0x0D
#define FLAG_STATUS           0x0E                // where the last bit is reserved for last packet = 0x0F
#define FLAG_STARDOME_STATUS  0x10       // where the last bit is reserved for last packet = 0x11
#define FLAG_CRYPTO           0x12                // where the last bit is reserved for last packet = 0x13
#define FLAG_STARDOME_CRYPTO  0x34       // where the last bit is reserved for last packet = 0x35

// Byte stuffing / framing characters
#define FRAME_BYTE      0x7E  // Marks start and end of a frame
#define ESCAPE_CHAR     0x7D  // Escapes special characters
#define STUFF_BYTE      0x20  // XOR mask for escaped bytes

#define SHARED_TOKEN 0xABCD

// #############################################################################
// ## Data Encoding Types
// #############################################################################

#define ENCODING_BINARY      0x00 // Raw binary data
#define ENCODING_ASCII       0x01 // ASCII text data
#define ENCODING_UTF8        0x02 // UTF-8 encoded text
#define ENCODING_JSON        0x03 // JSON formatted data
#define ENCODING_CBOR        0x04 // CBOR encoded data
#define ENCODING_BASE64      0x05 // Base64 encoded data
#define ENCODING_HEX         0x06 // Hexadecimal string data

// -- Composite TLV Data Types --
// The 'V' (Value) in each TLV item is encoded as specified.
#define ENCODING_TLV_BINARY  0x07 // TLV with binary values
#define ENCODING_TLV_ASCII   0x08 // TLV with ASCII string values
#define ENCODING_TLV_UTF8    0x09 // TLV with UTF-8 string values

// #############################################################################
// ## Data Structures
// #############################################################################

// Enhanced packet structure with encoding type identifier
typedef struct {
    uint8_t flags;
    uint16_t seq;
    uint16_t seq_length;     // Total number of packets in sequence
    uint32_t seq_size;       // Total amount of DATA to be sent in sequence
    uint16_t data_length;    // Length of data in this packet
    uint8_t encoding_type;   // Data encoding type identifier
    uint8_t data[MAX_DATA_SIZE]; // Variable-size data payload
    uint16_t token;
    uint16_t crc;            // CRC-16 checksum for error detection
} Packet;

// Receiver buffer slot
typedef struct {
    Packet pkt;
    bool received;
} BufferSlot;

// Enhanced sender buffer slot with MIN-inspired features
typedef struct {
    Packet pkt;
    bool sent;
    bool acked;
    uint32_t last_sent_time_ms;    // MIN-inspired: timeout tracking
    uint8_t retransmit_count;      // MIN-inspired: retry limiting
    uint16_t payload_ring_offset;  // MIN-inspired: efficient storage
} EnhancedSendSlot;

// Connection state management inspired by MIN
typedef struct {
    uint32_t last_received_anything_ms;
    uint32_t last_sent_ack_time_ms;
    uint32_t last_received_frame_ms;
    uint32_t connection_timeout_ms;
    bool connection_active;
    // Diagnostic counters from MIN
    uint32_t dropped_frames;
    uint32_t spurious_acks;
    uint32_t retransmissions;
    uint32_t sequence_mismatch_drop;
    uint32_t resets_received;
    uint16_t max_window_usage;     // Peak window utilization
} ConnectionState;

// #############################################################################
// ## Function Prototypes
// #############################################################################

// Initialization
void init_sliding_window_protocol(void);

// Reliable Send
void reliable_send_buffered(const uint8_t *data, uint16_t total_len);
void reliable_send_buffered_with_encoding(const uint8_t *data, uint16_t total_len, uint8_t encoding_type);

// Reliable Receive
// make output_len volatile so ISR-managed counters can be passed directly
void reliable_receive_buffered(uint8_t *output, volatile uint16_t *output_len);
void reliable_receive_buffered_with_encoding(uint8_t *output, volatile uint16_t *output_len, uint8_t *encoding_type);

// Public helper: try to get a decoded payload (nonblocking). Returns true when a payload was produced.
bool receive_decoded_frame(uint8_t *out, uint16_t *out_len);

// Diagnostics
void get_connection_statistics(ConnectionState *stats);
void reset_connection_statistics(void);

// Encoding Utilities
bool is_valid_encoding_type(uint8_t encoding_type);
const char* get_encoding_type_name(uint8_t encoding_type);

// #############################################################################
// ## UART and System Dependencies (to be implemented by user)
// #############################################################################

bool uart_send_byte(uint8_t byte);
uint8_t uart_receive_byte(void);

/* Should be a non-blocking boolean that tells the protocol whether at least one byte is waiting on RX.
 * It should not consume a byte, just report availability, because receive_any_frame() expects to poll
 * and then call uart_receive_byte() to actually read bytes.
 */
bool uart_RX_available(void);

uint32_t millis(void);
// Public wrapper: send a single-packet payload that matches the Packet structure
// The sliding-window will set internal seq and token.
void sliding_window_send_payload(const uint8_t *data, uint16_t len, uint8_t flags, uint8_t encoding_type);

#endif // SLIDING_WINDOW_PROTOCOL_16BIT_H
