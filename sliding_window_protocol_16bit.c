#include "sliding_window_protocol_16bit.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

// #############################################################################
// ## UART and System Dependencies (to be implemented by user)
// #############################################################################


// Provide weak, overrideable defaults for platform-specific hooks so the
// module links even if the application hasn't supplied implementations yet.
// Real platforms should provide their own (strong) implementations which
// will override these weak symbols at link time.
#include <stdint.h>
#include <stdbool.h>

__attribute__((weak)) bool uart_send_byte(uint8_t byte) {
    (void)byte;
    return false;
}

__attribute__((weak)) uint8_t uart_receive_byte(void) {
    return 0;
}

__attribute__((weak)) bool uart_RX_available(void) {
    return false;
}

__attribute__((weak)) uint32_t millis(void) {
    return 0U;
}

__attribute__((weak)) bool handle_flag_sign_frame(const uint8_t *data, uint16_t len) {
    (void)data;
    (void)len;
    return false;
}

// #############################################################################
// ## Private Variables
// #############################################################################

// Receiver-side buffer management
static BufferSlot recv_window[WINDOW_SIZE];
static uint16_t expected_frame_index = 0;

// Enhanced sender-side buffer management with MIN-inspired features
static EnhancedSendSlot send_window[WINDOW_SIZE];
static uint16_t base_frame_index = 0;
static uint16_t next_frame_index = 0;
static ConnectionState conn_state = {0};

typedef enum {
    FRAME_TYPE_DATA = 0,
    FRAME_TYPE_ACK,
    FRAME_TYPE_NACK,
    FRAME_TYPE_RESET,
    FRAME_TYPE_UNKNOWN
} FrameType;

// #############################################################################
// ## Private Function Prototypes
// #############################################################################

static uint16_t compute_crc16(const uint8_t *data, uint16_t len);
static void send_escaped_byte(uint8_t byte);
static void send_frame_struct(Frame *frame);
static bool receive_any_frame(FrameType *type, Frame *frame, uint8_t *ctrl_data, uint8_t *ctrl_len);
static void store_frame(Frame *frame);
static uint8_t generate_sack_window(void);
static void send_sack_ack(void);
static void send_nack(void);
static void process_in_order_frames(uint8_t *output, uint16_t *output_len);
static void init_connection_state(void);
static void update_connection_activity(void);
static bool is_connection_alive(void);
static void prepare_and_send_frame(const uint8_t *data, uint16_t len, uint8_t flags, uint16_t total_frames, uint32_t total_size, uint8_t encoding_type);
static void handle_sack_ack(uint16_t sack_base, uint8_t window_frames);
static void check_timeouts_and_retransmit(void);
static void handle_reset_frame(void);
static bool validate_frame_sequence(Frame *frame);

// #############################################################################
// ## Public Function Implementations
// #############################################################################

// Enhanced initialization with MIN concepts
void init_sliding_window_protocol(void) {
    init_connection_state();
    
    // Initialize send window
    for (int i = 0; i < WINDOW_SIZE; i++) {
        send_window[i].sent = false;
        send_window[i].acked = false;
        send_window[i].retransmit_count = 0;
        send_window[i].last_sent_time_ms = 0;
    }
    
    // Initialize receive window
    for (int i = 0; i < WINDOW_SIZE; i++) {
        recv_window[i].received = false;
    }
    
    base_frame_index = 0;
    next_frame_index = 0;
    expected_frame_index = 0;
}

void reliable_send_buffered(const uint8_t *data, uint16_t total_len) {
    reliable_send_buffered_with_encoding(data, total_len, ENCODING_BINARY);
}

void reliable_send_buffered_with_encoding(const uint8_t *data, uint16_t total_len, uint8_t encoding_type) {
    if (!is_valid_encoding_type(encoding_type)) {
        encoding_type = ENCODING_BINARY;  // Default to binary for invalid types
    }
    
    uint16_t offset = 0;
    uint16_t total_frames = (total_len + MAX_DATA_SIZE - 1) / MAX_DATA_SIZE;

    while (offset < total_len || base_frame_index != next_frame_index) {
        while (((next_frame_index - base_frame_index + MAX_FRAME_INDEX) % MAX_FRAME_INDEX) < WINDOW_SIZE &&
               offset < total_len) {
            uint16_t chunk_len = (total_len - offset > MAX_DATA_SIZE) ? MAX_DATA_SIZE : (total_len - offset);
            uint8_t flags = (offset + chunk_len >= total_len) ? FLAG_LAST_FRAME : 0;

            prepare_and_send_frame(&data[offset], chunk_len, flags, total_frames, total_len, encoding_type);
            offset += chunk_len;
        }

        // **Always try to receive ACKs with a short timeout**
        FrameType ftype;
        Frame dummy_frame;
        uint8_t ctrl_data[4];
        uint8_t ctrl_len = 0;

        if (receive_any_frame(&ftype, &dummy_frame, ctrl_data, &ctrl_len)) {
            if (ftype == FRAME_TYPE_ACK && ctrl_len >= 3) {
                uint16_t sack_base = ((uint16_t)ctrl_data[0] << 8) | ctrl_data[1];
                uint8_t window_frames = ctrl_data[2];
                handle_sack_ack(sack_base, window_frames);
            } else if (ftype == FRAME_TYPE_NACK) {
                // Optional: could trigger more aggressive retransmission if desired
            } else if (ftype == FRAME_TYPE_RESET) {
                handle_reset_frame();
            }
        }
    
        check_timeouts_and_retransmit();
        
        if (!is_connection_alive()) {
            handle_reset_frame();
        }
    }
}

void reliable_receive_buffered(uint8_t *output, volatile uint16_t *output_len) {
    uint8_t encoding_type; // Dummy variable
    reliable_receive_buffered_with_encoding(output, output_len, &encoding_type);
}

void reliable_receive_buffered_with_encoding(uint8_t *output, volatile uint16_t *output_len, uint8_t *encoding_type) {
    *output_len = 0;          // writing to a volatile target is fine
    *encoding_type = ENCODING_BINARY;
    bool done = false;

    while (!done) {
        FrameType ftype;
        Frame frame;
        uint8_t ctrl_data[4];
        uint8_t ctrl_len = 0;

        if (!receive_any_frame(&ftype, &frame, ctrl_data, &ctrl_len)) {
            // No valid frame / timeout / no data; upper layer may decide to retry
            continue;
        }

        if (ftype == FRAME_TYPE_DATA) {
            if (!is_valid_encoding_type(frame.encoding_type)) {
                conn_state.sequence_mismatch_drop++;
                send_nack();
                continue;
            }

            *encoding_type = frame.encoding_type;
            store_frame(&frame);
            send_sack_ack();
            uint16_t out_len_local = *output_len;
            process_in_order_frames(output, &out_len_local);
            *output_len = out_len_local;
            
            // If this frame requests signing, attempt to parse the assembled payload.
            // If the full assembled payload is available use it; otherwise fall back to the
            // frame's data buffer. The stardome handler will validate the CBOR format.
            if (frame.flags & FLAG_SIGN) {
                const uint8_t *parse_buf = NULL;
                uint16_t parse_len = 0;
                if (*output_len > 0) {
                    parse_buf = output;
                    parse_len = *output_len;
                } else {
                    parse_buf = frame.data;
                    parse_len = frame.data_length;
                }

                // Call into the stardome-specific handler; react to failures by
                // sending a NACK and a small error response payload (raw byte 0x00).
                // Also update a diagnostic counter so failures are visible.
                bool ok = handle_flag_sign_frame(parse_buf, parse_len);
                if (!ok) {
                    conn_state.sequence_mismatch_drop++; // record parsing failure
                    // Send protocol-level negative acknowledgement
                    send_nack();

                    // Send an application-level error response containing the
                    // raw error code byte 0x00 so the sender can detect the error.
                    uint8_t err_code = 0x00;
                    sliding_window_send_payload(&err_code, 1, FLAG_LAST_FRAME, ENCODING_BINARY);
                }
            }

            if (frame.flags & FLAG_LAST_FRAME) {
                done = true;
            }
        } else if (ftype == FRAME_TYPE_ACK && ctrl_len >= 3) {
            uint16_t sack_base = ((uint16_t)ctrl_data[0] << 8) | ctrl_data[1];
            uint8_t window_frames = ctrl_data[2];
            handle_sack_ack(sack_base, window_frames);
        } else if (ftype == FRAME_TYPE_NACK) {
            // Optional: handle NACK on receiver side if needed
        } else if (ftype == FRAME_TYPE_RESET) {
            handle_reset_frame();
        }
    }
}


// Public helper — try to decode a single frame and give back its payload.
// Non-blocking; returns true if a decoded payload is available.
bool receive_decoded_frame(uint8_t *out, uint16_t *out_len)
{
    FrameType ftype;
    Frame frame;
    uint8_t ctrl_data[10];
    uint8_t ctrl_len;

    // receive_any_frame returns true when a whole frame was decoded into frame.
    if (!receive_any_frame(&ftype, &frame, ctrl_data, &ctrl_len)) {
        return false;
    }

    // Only return data payloads; ignore control frames
    if (ftype != FRAME_TYPE_DATA) {
        return false;
    }

    memcpy(out, frame.data, frame.data_length);
    *out_len = frame.data_length;
    return true;
}

void get_connection_statistics(ConnectionState *stats) {
    *stats = conn_state;
}

void reset_connection_statistics(void) {
    conn_state.dropped_frames = 0;
    conn_state.spurious_acks = 0;
    conn_state.retransmissions = 0;
    conn_state.sequence_mismatch_drop = 0;
    conn_state.resets_received = 0;
    conn_state.max_window_usage = 0;
}

bool is_valid_encoding_type(uint8_t encoding_type) {
    // Check for base types (0x00-0x07, excluding 0x05)
    if (encoding_type <= ENCODING_HEX && encoding_type != 0x05) {
        return true;
    }
    // Check for composite TLV types (0x10-0x12)
    if (encoding_type >= ENCODING_TLV_BINARY && encoding_type <= ENCODING_TLV_UTF8) {
        return true;
    }
    return false;
}

const char* get_encoding_type_name(uint8_t encoding_type) {
    switch (encoding_type) {
        case ENCODING_BINARY: return "BINARY";
        case ENCODING_ASCII:  return "ASCII";
        case ENCODING_UTF8:   return "UTF-8";
        case ENCODING_JSON:   return "JSON";
        case ENCODING_CBOR:   return "CBOR";
        case ENCODING_BASE64: return "BASE64";
        case ENCODING_HEX:    return "HEX";
        case ENCODING_TLV_BINARY: return "TLV_BINARY";
        case ENCODING_TLV_ASCII:  return "TLV_ASCII";
        case ENCODING_TLV_UTF8:   return "TLV_UTF8";
        default: return "UNKNOWN";
    }
}

// #############################################################################
// ## Private Function Implementations
// #############################################################################

// CRC-16 calculation (CRC-16-CCITT, polynomial 0x1021)
static inline uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data) {
    crc ^= ((uint16_t)data << 8);

    for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x8000) {
            crc = (uint16_t)((crc << 1) ^ 0x1021);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static uint16_t compute_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;  // Initial value

    for (uint16_t i = 0; i < len; i++) {
        crc = crc16_ccitt_update(crc, data[i]);
    }

    return crc;
}

// #############################################################################
// ## Byte Stuffing Functions
// #############################################################################

// Sends a byte, escaping it if necessary
static void send_escaped_byte(uint8_t byte) {
    if (byte == FRAME_BYTE || byte == ESCAPE_CHAR) {
        uint8_t esc = ESCAPE_CHAR;
        uart_send_byte(esc);
        uart_send_byte(byte ^ STUFF_BYTE);
    } else {
        uart_send_byte(byte);
    }
}

// Send frame over UART with encoding type
static inline void le16_write(uint8_t *buf, uint16_t v) {
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void le32_write(uint8_t *buf, uint32_t v) {
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
    buf[2] = (uint8_t)((v >> 16) & 0xFF);
    buf[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void send_frame_struct(Frame *frame) {
    /*
     * Wire format
     * [FRAME_BYTE]
     *   flags (1) |
    *   frame_index (2) |
     *   seq_length (2) |
     *   seq_size (4) |
     *   data_length (2) |
     *   encoding_type (1) |
     *   data[...] |
     *   token (2) |
     *   crc (2)
     * [FRAME_BYTE]
     */

    uint8_t header[12];
    header[0] = frame->flags;
    le16_write(&header[1], frame->frame_index);
    le16_write(&header[3], frame->seq_length);
    le32_write(&header[5], frame->seq_size);
    le16_write(&header[9], frame->data_length);
    header[11] = frame->encoding_type;

    // Compute CRC incrementally to avoid large stack allocations.
    // CRC covers: header + payload + token (little-endian), excluding the CRC itself.
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < (uint16_t)sizeof(header); i++) {
        crc = crc16_ccitt_update(crc, header[i]);
    }
    for (uint16_t i = 0; i < frame->data_length; i++) {
        crc = crc16_ccitt_update(crc, frame->data[i]);
    }
    crc = crc16_ccitt_update(crc, (uint8_t)(frame->token & 0xFF));
    crc = crc16_ccitt_update(crc, (uint8_t)((frame->token >> 8) & 0xFF));
    frame->crc = crc;

    uart_send_byte(FRAME_BYTE);

    for (uint16_t i = 0; i < sizeof(header); i++) {
        send_escaped_byte(header[i]);
    }
    for (uint16_t i = 0; i < frame->data_length; i++) {
        send_escaped_byte(frame->data[i]);
    }
    send_escaped_byte((uint8_t)(frame->token & 0xFF));
    send_escaped_byte((uint8_t)((frame->token >> 8) & 0xFF));
    send_escaped_byte((uint8_t)(crc & 0xFF));
    send_escaped_byte((uint8_t)((crc >> 8) & 0xFF));

    uart_send_byte(FRAME_BYTE);
}

// Receive any frame (data or control) with timeout and byte-stuffing handling
static bool receive_any_frame(FrameType *type, Frame *frame,
                              uint8_t *ctrl_data, uint8_t *ctrl_len) {
    uint8_t byte;
    bool in_escape = false;
    uint32_t start_ms = millis();
    uint32_t last_byte_ms = start_ms;
    const uint32_t BYTE_TIMEOUT_MS = 100;      // Timeout between bytes after frame start
    const uint32_t FRAME_START_TIMEOUT_MS = 50; // Give up quickly if nothing is pending

    while (true) {
        if (!uart_RX_available()) {
            if ((millis() - start_ms) > FRAME_START_TIMEOUT_MS) {
                return false;
            }
            continue;
        }

        byte = uart_receive_byte();
        last_byte_ms = millis();
        if (byte == FRAME_BYTE) {
            break;
        }
    }

    while (true) {
        if (!uart_RX_available()) {
            if ((millis() - last_byte_ms) > BYTE_TIMEOUT_MS) {
                return false;
            }
            continue;
        }

        byte = uart_receive_byte();
        last_byte_ms = millis();

        if (in_escape) {
            byte ^= STUFF_BYTE;
            in_escape = false;
            break;
        }

        if (byte == ESCAPE_CHAR) {
            in_escape = true;
            continue;
        }

        if (byte == FRAME_BYTE) {
            return false; // Empty frame
        }

        break;
    }

    if (byte == ACK || byte == NACK || byte == RESET_FRAME) {
        *type = (byte == ACK) ? FRAME_TYPE_ACK :
                 (byte == NACK) ? FRAME_TYPE_NACK : FRAME_TYPE_RESET;

        uint8_t buffer[10];
        uint8_t bytes_received = 0;

        while (bytes_received < sizeof(buffer)) {
            if (!uart_RX_available()) {
                if ((millis() - last_byte_ms) > BYTE_TIMEOUT_MS) {
                    return false;
                }
                continue;
            }

            byte = uart_receive_byte();
            last_byte_ms = millis();

            if (in_escape) {
                buffer[bytes_received++] = byte ^ STUFF_BYTE;
                in_escape = false;
            } else if (byte == ESCAPE_CHAR) {
                in_escape = true;
            } else if (byte == FRAME_BYTE) {
                break;
            } else {
                buffer[bytes_received++] = byte;
            }
        }

        *ctrl_len = bytes_received;
        for (uint8_t i = 0; i < *ctrl_len; i++) {
            ctrl_data[i] = buffer[i];
        }
        return true;
    }

    *type = FRAME_TYPE_DATA;

    // Large buffer; keep off the stack to reduce overflow risk.
    static uint8_t frame_buf[SWP_MAX_FRAME_UNSTUFFED]; // header + data + token + crc
    uint16_t flen = 0;
    bool overflowed = false;
    uint16_t raw_count = 0;

    // First byte already read (after start delimiter).
    raw_count = 1U;
    if (raw_count > SWP_MAX_FRAME_STUFFED) {
        conn_state.dropped_frames++;
        return false;
    }
    frame_buf[flen++] = byte;

    while (true) {
        if (!uart_RX_available()) {
            if ((millis() - last_byte_ms) > BYTE_TIMEOUT_MS) {
                return false;
            }
            continue;
        }

        byte = uart_receive_byte();
        last_byte_ms = millis();

        if (byte != FRAME_BYTE) {
            raw_count++;
            if (raw_count > SWP_MAX_FRAME_STUFFED) {
                conn_state.dropped_frames++;
                return false;
            }
        }

        if (in_escape) {
            uint8_t unstuffed = (uint8_t)(byte ^ STUFF_BYTE);
            in_escape = false;
            if (!overflowed) {
                if (flen < sizeof(frame_buf)) {
                    frame_buf[flen++] = unstuffed;
                } else {
                    overflowed = true;
                    conn_state.dropped_frames++;
                }
            }
            continue;
        }

        if (byte == ESCAPE_CHAR) {
            in_escape = true;
            continue;
        }

        if (byte == FRAME_BYTE) {
            break;
        }

        if (!overflowed) {
            if (flen < sizeof(frame_buf)) {
                frame_buf[flen++] = byte;
            } else {
                overflowed = true;
                conn_state.dropped_frames++;
            }
        }
    }

    if (overflowed) {
        return false;
    }

    if (flen < (12 + 2 + 2)) {
        return false;
    }

    const uint8_t *h = frame_buf;
    frame->flags = h[0];
    frame->frame_index = (uint16_t)h[1] | ((uint16_t)h[2] << 8);
    frame->seq_length = (uint16_t)h[3] | ((uint16_t)h[4] << 8);
    frame->seq_size = (uint32_t)h[5] | ((uint32_t)h[6] << 8) | ((uint32_t)h[7] << 16) | ((uint32_t)h[8] << 24);
    frame->data_length = (uint16_t)h[9] | ((uint16_t)h[10] << 8);
    frame->encoding_type = h[11];

    if (frame->data_length > MAX_DATA_SIZE) {
        return false;
    }

    const uint16_t expected_len = (uint16_t)(12 + frame->data_length + 2 + 2);
    if (flen != expected_len) {
        return false;
    }

    for (uint16_t i = 0; i < frame->data_length; i++) {
        frame->data[i] = frame_buf[12 + i];
    }

    frame->token = (uint16_t)frame_buf[12 + frame->data_length] | ((uint16_t)frame_buf[12 + frame->data_length + 1] << 8);
    const uint16_t received_crc = (uint16_t)frame_buf[12 + frame->data_length + 2] | ((uint16_t)frame_buf[12 + frame->data_length + 3] << 8);

    const uint16_t computed_crc = compute_crc16(frame_buf, (uint16_t)(12 + frame->data_length + 2));
    if (computed_crc != received_crc) {
        return false;
    }

    if (frame->token != SHARED_TOKEN) {
        return false;
    }

    frame->crc = received_crc;

    if (!validate_frame_sequence(frame)) {
        return false;
    }

    update_connection_activity();
    conn_state.last_received_frame_ms = millis();
    return true;
}

static void store_frame(Frame *frame) {
    uint16_t index = (frame->frame_index - expected_frame_index + MAX_FRAME_INDEX) % MAX_FRAME_INDEX;
    if (index < WINDOW_SIZE && !recv_window[index].received) {
        recv_window[index].frame = *frame;
        recv_window[index].received = true;
    }
}

static uint8_t generate_sack_window(void) {
    uint8_t window_frames = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        if (recv_window[i].received) {
            window_frames |= (1 << i);
        }
    }
    return window_frames;
}

static void send_sack_ack(void) {
    uint8_t window_frames = generate_sack_window();
    
    // Send ACK frame with byte stuffing
    uart_send_byte(FRAME_BYTE);
    send_escaped_byte(ACK);
    send_escaped_byte((expected_frame_index >> 8) & 0xFF);
    send_escaped_byte(expected_frame_index & 0xFF);
    send_escaped_byte(window_frames);
    uart_send_byte(FRAME_BYTE);
    
    conn_state.last_sent_ack_time_ms = millis();
}

static void send_nack(void) {
    // Send NACK frame with byte stuffing
    uart_send_byte(FRAME_BYTE);
    send_escaped_byte(NACK);
    uart_send_byte(FRAME_BYTE);
}

static void process_in_order_frames(uint8_t *output, uint16_t *output_len) {
    while (recv_window[0].received && recv_window[0].frame.frame_index == expected_frame_index) {
        Frame *frame = &recv_window[0].frame;
        memcpy(&output[*output_len], frame->data, frame->data_length);
        *output_len += frame->data_length;

        if (frame->flags & FLAG_LAST_FRAME) {
            break;
        }

        for (int i = 1; i < WINDOW_SIZE; i++) {
            recv_window[i - 1] = recv_window[i];
        }
        recv_window[WINDOW_SIZE - 1].received = false;
        expected_frame_index = (expected_frame_index + 1) % MAX_FRAME_INDEX;
    }
}

static void init_connection_state(void) {
    conn_state.connection_timeout_ms = CONNECTION_TIMEOUT_MS;
    conn_state.connection_active = false;
    conn_state.dropped_frames = 0;
    conn_state.spurious_acks = 0;
    conn_state.retransmissions = 0;
    conn_state.sequence_mismatch_drop = 0;
    conn_state.resets_received = 0;
    conn_state.max_window_usage = 0;
}

static void update_connection_activity(void) {
    conn_state.last_received_anything_ms = millis();
    conn_state.connection_active = true;
}

static bool is_connection_alive(void) {
    uint32_t now = millis();
    return (now - conn_state.last_received_anything_ms) < conn_state.connection_timeout_ms;
}

static void prepare_and_send_frame(const uint8_t *data, uint16_t len, uint8_t flags, uint16_t total_frames, uint32_t total_size, uint8_t encoding_type) {
    uint8_t index = next_frame_index % WINDOW_SIZE;

    Frame *frame = &send_window[index].frame;
    frame->flags = flags;
    frame->frame_index = next_frame_index;
    frame->seq_length = total_frames;
    frame->seq_size = total_size;
    frame->data_length = len;
    frame->encoding_type = encoding_type;
    memcpy(frame->data, data, len);
    frame->token = SHARED_TOKEN;

    send_frame_struct(frame);

    send_window[index].sent = true;
    send_window[index].acked = false;
    send_window[index].last_sent_time_ms = millis();
    send_window[index].retransmit_count = 0;

    next_frame_index = (next_frame_index + 1) % MAX_FRAME_INDEX;
    
    uint8_t current_window_usage = ((next_frame_index - base_frame_index + MAX_FRAME_INDEX) % MAX_FRAME_INDEX);
    if (current_window_usage > conn_state.max_window_usage) {
        conn_state.max_window_usage = current_window_usage;
    }
}

// Public wrapper: send a single-frame payload directly through the sliding-window
// This constructs a single frame and sends it through the existing sender logic.
void sliding_window_send_payload(const uint8_t *data, uint16_t len, uint8_t flags, uint8_t encoding_type) {
    // For a single frame, total_frames = 1 and total_size = len
    prepare_and_send_frame(data, len, flags, 1, len, encoding_type);
}

// Helper to send large payloads by fragmenting them into multiple frames
// Blocks until all fragments are queued (and waits for window space if needed)
void sliding_window_send_fragmented(const uint8_t *data, uint32_t len, uint8_t flags, uint8_t encoding_type) {
    uint16_t total_frames = (uint16_t)((len + MAX_DATA_SIZE - 1) / MAX_DATA_SIZE);
    uint32_t offset = 0;
    
    for (uint16_t i = 0; i < total_frames; i++) {
        // Wait for window space: block while the window is full
        // (next_frame_index - base_frame_index) >= WINDOW_SIZE
        while ( ((next_frame_index - base_frame_index + MAX_FRAME_INDEX) % MAX_FRAME_INDEX) >= WINDOW_SIZE ) {
             // Poll for ACKs to advance window
             Frame rx_frame;
             FrameType ftype;
             uint8_t ctrl_data[16];
             uint8_t ctrl_len = 0;
             
             // We must process incoming ACKs to free up window slots
             if (receive_any_frame(&ftype, &rx_frame, ctrl_data, &ctrl_len)) {
                if (ftype == FRAME_TYPE_ACK && ctrl_len >= 3) {
                    uint16_t sack_base = ((uint16_t)ctrl_data[0] << 8) | ctrl_data[1];
                    uint8_t window_frames = ctrl_data[2];
                    handle_sack_ack(sack_base, window_frames);
                } else if (ftype == FRAME_TYPE_RESET) {
                    handle_reset_frame();
                    // If reset, we probably should abort, but for now we just continue 
                    // (next_frame_index/base_frame_index reset to 0, so loop condition will clear)
                }
             }
             check_timeouts_and_retransmit();
             
             // Optional: yield or small delay if OS present, but here we busy-wait
        }

        uint16_t chunk_len = (uint16_t)((len - offset > MAX_DATA_SIZE) ? MAX_DATA_SIZE : (len - offset));
        
        // If this is the last chunk, preserve the original flags (e.g. FLAG_LAST_FRAME)
        // Otherwise, mask out FLAG_LAST_FRAME because more chunks follow.
        uint8_t current_flags = flags;
        if (i < total_frames - 1) {
            current_flags &= ~FLAG_LAST_FRAME;
        }
        
        prepare_and_send_frame(data + offset, chunk_len, current_flags, total_frames, len, encoding_type);
        offset += chunk_len;
    }
}

static void handle_sack_ack(uint16_t sack_base, uint8_t window_frames) {
    bool valid_ack = false;
    
    for (int i = 0; i < WINDOW_SIZE; i++) {
        uint16_t frame_index = (sack_base + i) % MAX_FRAME_INDEX;
        uint8_t index = frame_index % WINDOW_SIZE;

        if (window_frames & (1 << i)) {
            if (send_window[index].sent && !send_window[index].acked) {
                send_window[index].acked = true;
                valid_ack = true;
            } else {
                conn_state.spurious_acks++;
            }
        }
    }

    if (valid_ack) {
        update_connection_activity();
    }

    while (send_window[base_frame_index % WINDOW_SIZE].acked) {
        send_window[base_frame_index % WINDOW_SIZE].sent = false;
        send_window[base_frame_index % WINDOW_SIZE].acked = false;
        send_window[base_frame_index % WINDOW_SIZE].retransmit_count = 0;
        base_frame_index = (base_frame_index + 1) % MAX_FRAME_INDEX;
    }
}

static void check_timeouts_and_retransmit(void) {
    uint32_t now = millis();
    
    for (int i = 0; i < WINDOW_SIZE; i++) {
        if (send_window[i].sent && !send_window[i].acked) {
            uint32_t timeout = TIMEOUT_MS * (1 << send_window[i].retransmit_count);
            
            if ((now - send_window[i].last_sent_time_ms) > timeout) {
                if (send_window[i].retransmit_count < MAX_RETRANSMIT_COUNT) {
                    send_frame_struct(&send_window[i].frame);
                    send_window[i].last_sent_time_ms = now;
                    send_window[i].retransmit_count++;
                    conn_state.retransmissions++;
                } else {
                    send_window[i].sent = false;
                    send_window[i].acked = false;
                    conn_state.dropped_frames++;
                    
                    uart_send_byte(RESET_FRAME);
                }
            }
        }
    }
}

static void handle_reset_frame(void) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
        send_window[i].sent = false;
        send_window[i].acked = false;
        send_window[i].retransmit_count = 0;
    }
    
    base_frame_index = 0;
    next_frame_index = 0;
    expected_frame_index = 0;
    
    for (int i = 0; i < WINDOW_SIZE; i++) {
        recv_window[i].received = false;
    }
    
    conn_state.resets_received++;
    conn_state.connection_active = false;
}

static bool validate_frame_sequence(Frame *frame) {
    uint16_t seq_diff = (frame->frame_index - expected_frame_index + MAX_FRAME_INDEX) % MAX_FRAME_INDEX;
    
    if (seq_diff >= WINDOW_SIZE) {
        conn_state.sequence_mismatch_drop++;
        return false;
    }
    
    return true;
}
