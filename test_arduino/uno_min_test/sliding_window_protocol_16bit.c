// Full protocol implementation copied from project root, unchanged except for small-footprint settings via header macros.
#include "sliding_window_protocol_16bit.h"
#include <string.h>

// Optional UNO instrumentation hooks implemented by the sketch
extern void uart_flush(void);
extern void swp_debug_pulse(uint8_t code);

// #############################################################################
// ## UART and System Dependencies (to be implemented by user)
// #############################################################################

// bool uart_send_byte(uint8_t byte);
// uint8_t uart_receive_byte(void);
// bool uart_RX_available(void);
// uint32_t millis(void);

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
    for (int i = 0; i < WINDOW_SIZE; i++) {
        send_window[i].sent = false;
        send_window[i].acked = false;
        send_window[i].retransmit_count = 0;
        send_window[i].last_sent_time_ms = 0;
    }
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
        encoding_type = ENCODING_BINARY;
    }

    uint16_t offset = 0;
    uint16_t total_frames = (total_len + MAX_DATA_SIZE - 1) / MAX_DATA_SIZE;

    while (offset < total_len || base_frame_index != next_frame_index) {
        while (((next_frame_index - base_frame_index + MAX_FRAME_INDEX) % MAX_FRAME_INDEX) < WINDOW_SIZE &&
               offset < total_len) {
            uint16_t remaining = (uint16_t)(total_len - offset);
            uint16_t chunk_len = (remaining > MAX_DATA_SIZE) ? MAX_DATA_SIZE : remaining;
            uint8_t flags = (offset + chunk_len >= total_len) ? FLAG_LAST_FRAME : 0;

            prepare_and_send_frame(&data[offset], chunk_len, flags, total_frames, total_len, encoding_type);
            offset = (uint16_t)(offset + chunk_len);
        }

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
                // No special handling required, parity with desktop implementation
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
    uint8_t encoding_type;
    reliable_receive_buffered_with_encoding(output, output_len, &encoding_type);
}

void reliable_receive_buffered_with_encoding(uint8_t *output, volatile uint16_t *output_len, uint8_t *encoding_type) {
    *output_len = 0;
    *encoding_type = ENCODING_BINARY;
    bool done = false;

    while (!done) {
        FrameType ftype;
        Frame frame;
        uint8_t ctrl_data[4];
        uint8_t ctrl_len = 0;

        if (!receive_any_frame(&ftype, &frame, ctrl_data, &ctrl_len)) {
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

            // Indicate a full message was received and assembled (code 2 = assembled)
            swp_debug_pulse(2);

            if (frame.flags & FLAG_LAST_FRAME) {
                done = true;
            }
        } else if (ftype == FRAME_TYPE_ACK && ctrl_len >= 3) {
            uint16_t sack_base = ((uint16_t)ctrl_data[0] << 8) | ctrl_data[1];
            uint8_t window_frames = ctrl_data[2];
            handle_sack_ack(sack_base, window_frames);
        } else if (ftype == FRAME_TYPE_NACK) {
            // Optional: NACK handling not required on receiver for UNO build
        } else if (ftype == FRAME_TYPE_RESET) {
            handle_reset_frame();
        }
    }
}

bool receive_decoded_frame(uint8_t *out, uint16_t *out_len) {
    FrameType ftype;
    Frame frame;
    uint8_t ctrl_data[10];
    uint8_t ctrl_len;
    if (!receive_any_frame(&ftype, &frame, ctrl_data, &ctrl_len)) {
        return false;
    }
    if (ftype != FRAME_TYPE_DATA) {
        return false;
    }
    memcpy(out, frame.data, frame.data_length);
    *out_len = frame.data_length;
    return true;
}

void get_connection_statistics(ConnectionState *stats) { *stats = conn_state; }
void reset_connection_statistics(void) {
    conn_state.dropped_frames = 0;
    conn_state.spurious_acks = 0;
    conn_state.retransmissions = 0;
    conn_state.sequence_mismatch_drop = 0;
    conn_state.resets_received = 0;
    conn_state.max_window_usage = 0;
}

bool is_valid_encoding_type(uint8_t encoding_type) {
    if (encoding_type <= ENCODING_HEX && encoding_type != 0x05) return true;
    if (encoding_type >= ENCODING_TLV_BINARY && encoding_type <= ENCODING_TLV_UTF8) return true;
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

// CRC-16 calculation (CRC-16-CCITT, polynomial 0x1021)
static uint16_t compute_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; else crc = crc << 1;
        }
    }
    return crc;
}

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
    // Build wire format: header(12) + data(data_length) + token(2) + crc(2)
    uint8_t header[12];
    header[0] = frame->flags;
    le16_write(&header[1], frame->frame_index);
    le16_write(&header[3], frame->seq_length);
    le32_write(&header[5], frame->seq_size);
    le16_write(&header[9], frame->data_length);
    header[11] = frame->encoding_type;

    const uint16_t crc_input_len = (uint16_t)(sizeof(header) + frame->data_length + sizeof(frame->token));
    uint8_t crc_buf[12 + MAX_DATA_SIZE + sizeof(uint16_t)];

    memcpy(crc_buf, header, sizeof(header));
    memcpy(&crc_buf[sizeof(header)], frame->data, frame->data_length);
    le16_write(&crc_buf[sizeof(header) + frame->data_length], frame->token);

    const uint16_t crc = compute_crc16(crc_buf, crc_input_len);
    frame->crc = crc;

    // Send framed with byte-stuffing
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

    // Indicate an outbound frame was sent (code 3 = frame sent)
    swp_debug_pulse(3);
    // Flush after sending a full frame so USB CDC pushes immediately
    uart_flush();
}

// Receive any frame (data or control) with timeout and byte-stuffing handling
static bool receive_any_frame(FrameType *type, Frame *frame, uint8_t *ctrl_data, uint8_t *ctrl_len) {
    uint8_t byte;
    bool in_escape = false;
    bool saw_any_byte = false;
    bool frame_started = false;
    uint32_t start_ms = millis();
    uint32_t last_byte_ms = start_ms;
    const uint32_t BYTE_TIMEOUT_MS = 100;
    const uint32_t FRAME_START_TIMEOUT_MS = 50;
    while (true) {
        if (!uart_RX_available()) {
            if ((millis() - start_ms) > FRAME_START_TIMEOUT_MS) {
                if (saw_any_byte) {
                    swp_debug_pulse(11);
                }
                return false;
            }
            continue;
        }
        byte = uart_receive_byte();
        saw_any_byte = true;
        last_byte_ms = millis();
        if (byte == FRAME_BYTE) break;
    }
    frame_started = true;
    while (true) {
        if (!uart_RX_available()) {
            if ((millis() - last_byte_ms) > BYTE_TIMEOUT_MS) {
                if (frame_started) {
                    swp_debug_pulse(12);
                }
                return false;
            }
            continue;
        }
        byte = uart_receive_byte();
        last_byte_ms = millis();
        if (in_escape) { byte ^= STUFF_BYTE; in_escape = false; break; }
        else if (byte == ESCAPE_CHAR) { in_escape = true; continue; }
        else if (byte == FRAME_BYTE) { return false; }
        else { break; }
    }
    if (byte == ACK || byte == NACK || byte == RESET_FRAME) {
        *type = (byte == ACK) ? FRAME_TYPE_ACK : (byte == NACK) ? FRAME_TYPE_NACK : FRAME_TYPE_RESET;
        uint8_t buffer[10];
        uint8_t bytes_received = 0;
        while (bytes_received < sizeof(buffer)) {
            if (!uart_RX_available()) {
                if ((millis() - last_byte_ms) > BYTE_TIMEOUT_MS) {
                    if (frame_started) {
                        swp_debug_pulse(12);
                    }
                    return false;
                }
                continue;
            }
            byte = uart_receive_byte();
            last_byte_ms = millis();
            if (in_escape) { buffer[bytes_received++] = byte ^ STUFF_BYTE; in_escape = false; }
            else if (byte == ESCAPE_CHAR) { in_escape = true; }
            else if (byte == FRAME_BYTE) { break; }
            else { buffer[bytes_received++] = byte; }
        }
        *ctrl_len = bytes_received;
        for (uint8_t i = 0; i < *ctrl_len; i++) ctrl_data[i] = buffer[i];
        return true;
    }
    *type = FRAME_TYPE_DATA;
    // Collect full frame payload (up to max allowed)
    uint8_t frame_buf[12 + MAX_DATA_SIZE + 4]; // header+data+token+crc
    uint16_t flen = 0;
    // First byte we already read into 'byte'
    // Interpret it as header[0]
    frame_buf[flen++] = byte;
    while (flen < sizeof(frame_buf)) {
        if (!uart_RX_available()) {
            if ((millis() - last_byte_ms) > BYTE_TIMEOUT_MS) {
                if (frame_started) {
                    swp_debug_pulse(12);
                }
                return false;
            }
            continue;
        }
        byte = uart_receive_byte();
        last_byte_ms = millis();
        if (in_escape) { frame_buf[flen++] = byte ^ STUFF_BYTE; in_escape = false; }
        else if (byte == ESCAPE_CHAR) { in_escape = true; }
        else if (byte == FRAME_BYTE) { break; }
        else { frame_buf[flen++] = byte; }
    }
    if (flen < (12 + 2 + 2)) return false; // minimal header+token+crc

    // Parse header (LE)
    const uint8_t *h = frame_buf;
    frame->flags = h[0];
    frame->frame_index = (uint16_t)h[1] | ((uint16_t)h[2] << 8);
    frame->seq_length = (uint16_t)h[3] | ((uint16_t)h[4] << 8);
    frame->seq_size = (uint32_t)h[5] | ((uint32_t)h[6] << 8) | ((uint32_t)h[7] << 16) | ((uint32_t)h[8] << 24);
    frame->data_length = (uint16_t)h[9] | ((uint16_t)h[10] << 8);
    frame->encoding_type = h[11];
    if (frame->data_length > MAX_DATA_SIZE) {
        swp_debug_pulse(6); // data length overflow
        return false;
    }
    const uint16_t expected_len = (uint16_t)(12 + frame->data_length + 2 + 2);
    if (flen != expected_len) {
        swp_debug_pulse(7); // frame length mismatch
        return false;
    }

    // Extract data
    for (uint16_t i = 0; i < frame->data_length; i++) frame->data[i] = frame_buf[12 + i];
    // Extract token (LE)
    frame->token = (uint16_t)frame_buf[12 + frame->data_length] | ((uint16_t)frame_buf[12 + frame->data_length + 1] << 8);
    // Extract CRC (LE)
    const uint16_t received_crc = (uint16_t)frame_buf[12 + frame->data_length + 2] | ((uint16_t)frame_buf[12 + frame->data_length + 3] << 8);

    // Compute CRC over header+data+token
    const uint16_t computed_crc = compute_crc16(frame_buf, (uint16_t)(12 + frame->data_length + 2));
    if (computed_crc != received_crc) {
        swp_debug_pulse(8); // CRC mismatch
        return false;
    }
    if (frame->token != SHARED_TOKEN) {
        swp_debug_pulse(9); // token mismatch
        return false;
    }
    frame->crc = received_crc;
    if (!validate_frame_sequence(frame)) {
        swp_debug_pulse(10); // sequence validation failure
        return false;
    }
    update_connection_activity();
    conn_state.last_received_frame_ms = millis();
    swp_debug_pulse(5); // data frame received and validated
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
        if (recv_window[i].received) window_frames |= (1 << i);
    }
    return window_frames;
}

static void send_sack_ack(void) {
    uint8_t window_frames = generate_sack_window();
    uart_send_byte(FRAME_BYTE);
    send_escaped_byte(ACK);
    send_escaped_byte((expected_frame_index >> 8) & 0xFF);
    send_escaped_byte(expected_frame_index & 0xFF);
    send_escaped_byte(window_frames);
    uart_send_byte(FRAME_BYTE);
    conn_state.last_sent_ack_time_ms = millis();
    // Ensure host receives bytes promptly
    uart_flush();
    // Visual debug: indicate ACK was sent (code 1)
    swp_debug_pulse(1);
}

static void send_nack(void) {
    uart_send_byte(FRAME_BYTE);
    send_escaped_byte(NACK);
    uart_send_byte(FRAME_BYTE);
}

static void process_in_order_frames(uint8_t *output, uint16_t *output_len) {
    while (recv_window[0].received && recv_window[0].frame.frame_index == expected_frame_index) {
        Frame *frame = &recv_window[0].frame;
        memcpy(&output[*output_len], frame->data, frame->data_length);
        *output_len += frame->data_length;
        if (frame->flags & FLAG_LAST_FRAME) break;
        for (int i = 1; i < WINDOW_SIZE; i++) recv_window[i - 1] = recv_window[i];
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
    if (current_window_usage > conn_state.max_window_usage) conn_state.max_window_usage = current_window_usage;
}

void sliding_window_send_payload(const uint8_t *data, uint16_t len, uint8_t flags, uint8_t encoding_type) {
    prepare_and_send_frame(data, len, flags, 1, len, encoding_type);
}

static void handle_sack_ack(uint16_t sack_base, uint8_t window_frames) {
    bool valid_ack = false;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        uint16_t frame_index = (sack_base + i) % MAX_FRAME_INDEX;
        uint8_t index = frame_index % WINDOW_SIZE;
        if (window_frames & (1 << i)) {
            if (send_window[index].sent && !send_window[index].acked) {
                send_window[index].acked = true; valid_ack = true;
            } else { conn_state.spurious_acks++; }
        }
    }
    if (valid_ack) update_connection_activity();
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
    base_frame_index = 0; next_frame_index = 0; expected_frame_index = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) recv_window[i].received = false;
    conn_state.resets_received++;
    conn_state.connection_active = false;
}

static bool validate_frame_sequence(Frame *frame) {
    uint16_t seq_diff = (frame->frame_index - expected_frame_index + MAX_FRAME_INDEX) % MAX_FRAME_INDEX;
    if (seq_diff >= WINDOW_SIZE) { conn_state.sequence_mismatch_drop++; return false; }
    return true;
}
