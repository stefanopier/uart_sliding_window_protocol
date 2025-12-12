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


// #############################################################################
// ## Private Variables
// #############################################################################

// Receiver-side buffer management
static BufferSlot recv_window[WINDOW_SIZE];
static uint16_t expected_seq = 0;

// Enhanced sender-side buffer management with MIN-inspired features
static EnhancedSendSlot send_window[WINDOW_SIZE];
static uint16_t base_seq = 0;
static uint16_t next_seq = 0;
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
static void send_packet_struct(Packet *pkt);
static bool receive_any_frame(FrameType *type, Packet *pkt, uint8_t *ctrl_data, uint8_t *ctrl_len);
static void store_packet(Packet *pkt);
static uint8_t generate_sack_bitmap(void);
static void send_sack_ack(void);
static void send_nack(void);
static void process_in_order_packets(uint8_t *output, uint16_t *output_len);
static void init_connection_state(void);
static void update_connection_activity(void);
static bool is_connection_alive(void);
static void prepare_and_send_packet(const uint8_t *data, uint16_t len, uint8_t flags, uint16_t total_packets, uint32_t total_size, uint8_t encoding_type);
static void handle_sack_ack(uint16_t sack_base, uint8_t bitmap);
static void check_timeouts_and_retransmit(void);
static void handle_reset_frame(void);
static bool validate_packet_sequence(Packet *pkt);

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
    
    base_seq = 0;
    next_seq = 0;
    expected_seq = 0;
}

void reliable_send_buffered(const uint8_t *data, uint16_t total_len) {
    reliable_send_buffered_with_encoding(data, total_len, ENCODING_BINARY);
}

void reliable_send_buffered_with_encoding(const uint8_t *data, uint16_t total_len, uint8_t encoding_type) {
    if (!is_valid_encoding_type(encoding_type)) {
        encoding_type = ENCODING_BINARY;  // Default to binary for invalid types
    }
    
    uint16_t offset = 0;
    uint16_t total_packets = (total_len + MAX_DATA_SIZE - 1) / MAX_DATA_SIZE;

    while (offset < total_len || base_seq != next_seq) {
        while (((next_seq - base_seq + MAX_SEQ_NUM) % MAX_SEQ_NUM) < WINDOW_SIZE &&
               offset < total_len) {
            uint16_t chunk_len = (total_len - offset > MAX_DATA_SIZE) ? MAX_DATA_SIZE : (total_len - offset);
            uint8_t flags = (offset + chunk_len >= total_len) ? FLAG_LAST_PACKET : 0;

            prepare_and_send_packet(&data[offset], chunk_len, flags, total_packets, total_len, encoding_type);
            offset += chunk_len;
        }

        // **Always try to receive ACKs with a short timeout**
        FrameType ftype;
        Packet dummy_pkt;
        uint8_t ctrl_data[4];
        uint8_t ctrl_len = 0;

        if (receive_any_frame(&ftype, &dummy_pkt, ctrl_data, &ctrl_len)) {
            if (ftype == FRAME_TYPE_ACK && ctrl_len >= 3) {
                uint16_t sack_base = ((uint16_t)ctrl_data[0] << 8) | ctrl_data[1];
                uint8_t bitmap = ctrl_data[2];
                handle_sack_ack(sack_base, bitmap);
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
        Packet pkt;
        uint8_t ctrl_data[4];
        uint8_t ctrl_len = 0;

        if (!receive_any_frame(&ftype, &pkt, ctrl_data, &ctrl_len)) {
            // No valid frame / timeout / no data; upper layer may decide to retry
            continue;
        }

        if (ftype == FRAME_TYPE_DATA) {
            if (!is_valid_encoding_type(pkt.encoding_type)) {
                conn_state.sequence_mismatch_drop++;
                send_nack();
                continue;
            }

            *encoding_type = pkt.encoding_type;
            store_packet(&pkt);
            send_sack_ack();
            uint16_t out_len_local = *output_len;
            process_in_order_packets(output, &out_len_local);
            *output_len = out_len_local;
            
            // If this packet requests signing, attempt to parse the assembled payload.
            // If the full assembled payload is available use it; otherwise fall back to the
            // packet's data buffer. The stardome handler will validate the CBOR format.
            if (pkt.flags & FLAG_SIGN) {
                const uint8_t *parse_buf = NULL;
                uint16_t parse_len = 0;
                if (*output_len > 0) {
                    parse_buf = output;
                    parse_len = *output_len;
                } else {
                    parse_buf = pkt.data;
                    parse_len = pkt.data_length;
                }

                // Call into the stardome-specific handler; react to failures by
                // sending a NACK and a small error response payload (raw byte 0x00).
                // Also update a diagnostic counter so failures are visible.
                bool ok = handle_flag_sign_packet(parse_buf, parse_len);
                if (!ok) {
                    conn_state.sequence_mismatch_drop++; // record parsing failure
                    // Send protocol-level negative acknowledgement
                    send_nack();

                    // Send an application-level error response containing the
                    // raw error code byte 0x00 so the sender can detect the error.
                    uint8_t err_code = 0x00;
                    sliding_window_send_payload(&err_code, 1, FLAG_LAST_PACKET, ENCODING_BINARY);
                }
            }

            if (pkt.flags & FLAG_LAST_PACKET) {
                done = true;
            }
        } else if (ftype == FRAME_TYPE_ACK && ctrl_len >= 3) {
            uint16_t sack_base = ((uint16_t)ctrl_data[0] << 8) | ctrl_data[1];
            uint8_t bitmap = ctrl_data[2];
            handle_sack_ack(sack_base, bitmap);
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
    Packet pkt;
    uint8_t ctrl_data[10];
    uint8_t ctrl_len;

    // receive_any_frame returns true when a whole frame was decoded into pkt.
    if (!receive_any_frame(&ftype, &pkt, ctrl_data, &ctrl_len)) {
        return false;
    }

    // Only return data payloads; ignore control frames
    if (ftype != FRAME_TYPE_DATA) {
        return false;
    }

    memcpy(out, pkt.data, pkt.data_length);
    *out_len = pkt.data_length;
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
static uint16_t compute_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;  // Initial value
    
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  // Polynomial
            } else {
                crc = crc << 1;
            }
        }
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

// Send packet over UART with encoding type
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

static void send_packet_struct(Packet *pkt) {
    /*
     * Wire format
     * [FRAME_BYTE]
     *   flags (1) |
     *   seq (2) |
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
    header[0] = pkt->flags;
    le16_write(&header[1], pkt->seq);
    le16_write(&header[3], pkt->seq_length);
    le32_write(&header[5], pkt->seq_size);
    le16_write(&header[9], pkt->data_length);
    header[11] = pkt->encoding_type;

    const uint16_t crc_input_len = (uint16_t)(sizeof(header) + pkt->data_length + sizeof(pkt->token));
    uint8_t crc_buf[12 + MAX_DATA_SIZE + sizeof(uint16_t)];

    memcpy(crc_buf, header, sizeof(header));
    memcpy(&crc_buf[sizeof(header)], pkt->data, pkt->data_length);
    le16_write(&crc_buf[sizeof(header) + pkt->data_length], pkt->token);

    const uint16_t crc = compute_crc16(crc_buf, crc_input_len);
    pkt->crc = crc;

    uart_send_byte(FRAME_BYTE);

    for (uint16_t i = 0; i < sizeof(header); i++) {
        send_escaped_byte(header[i]);
    }
    for (uint16_t i = 0; i < pkt->data_length; i++) {
        send_escaped_byte(pkt->data[i]);
    }
    send_escaped_byte((uint8_t)(pkt->token & 0xFF));
    send_escaped_byte((uint8_t)((pkt->token >> 8) & 0xFF));
    send_escaped_byte((uint8_t)(crc & 0xFF));
    send_escaped_byte((uint8_t)((crc >> 8) & 0xFF));

    uart_send_byte(FRAME_BYTE);
}

// Receive any frame (data or control) with timeout and byte-stuffing handling
static bool receive_any_frame(FrameType *type, Packet *pkt,
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

    uint8_t frame[12 + MAX_DATA_SIZE + 4]; // header + data + token + crc
    uint16_t flen = 0;
    frame[flen++] = byte;

    while (flen < sizeof(frame)) {
        if (!uart_RX_available()) {
            if ((millis() - last_byte_ms) > BYTE_TIMEOUT_MS) {
                return false;
            }
            continue;
        }

        byte = uart_receive_byte();
        last_byte_ms = millis();

        if (in_escape) {
            frame[flen++] = byte ^ STUFF_BYTE;
            in_escape = false;
        } else if (byte == ESCAPE_CHAR) {
            in_escape = true;
        } else if (byte == FRAME_BYTE) {
            break;
        } else {
            frame[flen++] = byte;
        }
    }

    if (flen < (12 + 2 + 2)) {
        return false;
    }

    const uint8_t *h = frame;
    pkt->flags = h[0];
    pkt->seq = (uint16_t)h[1] | ((uint16_t)h[2] << 8);
    pkt->seq_length = (uint16_t)h[3] | ((uint16_t)h[4] << 8);
    pkt->seq_size = (uint32_t)h[5] | ((uint32_t)h[6] << 8) | ((uint32_t)h[7] << 16) | ((uint32_t)h[8] << 24);
    pkt->data_length = (uint16_t)h[9] | ((uint16_t)h[10] << 8);
    pkt->encoding_type = h[11];

    if (pkt->data_length > MAX_DATA_SIZE) {
        return false;
    }

    const uint16_t expected_len = (uint16_t)(12 + pkt->data_length + 2 + 2);
    if (flen != expected_len) {
        return false;
    }

    for (uint16_t i = 0; i < pkt->data_length; i++) {
        pkt->data[i] = frame[12 + i];
    }

    pkt->token = (uint16_t)frame[12 + pkt->data_length] | ((uint16_t)frame[12 + pkt->data_length + 1] << 8);
    const uint16_t received_crc = (uint16_t)frame[12 + pkt->data_length + 2] | ((uint16_t)frame[12 + pkt->data_length + 3] << 8);

    const uint16_t computed_crc = compute_crc16(frame, (uint16_t)(12 + pkt->data_length + 2));
    if (computed_crc != received_crc) {
        return false;
    }

    if (pkt->token != SHARED_TOKEN) {
        return false;
    }

    pkt->crc = received_crc;

    if (!validate_packet_sequence(pkt)) {
        return false;
    }

    update_connection_activity();
    conn_state.last_received_frame_ms = millis();
    return true;
}

static void store_packet(Packet *pkt) {
    uint16_t index = (pkt->seq - expected_seq + MAX_SEQ_NUM) % MAX_SEQ_NUM;
    if (index < WINDOW_SIZE && !recv_window[index].received) {
        recv_window[index].pkt = *pkt;
        recv_window[index].received = true;
    }
}

static uint8_t generate_sack_bitmap(void) {
    uint8_t bitmap = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        if (recv_window[i].received) {
            bitmap |= (1 << i);
        }
    }
    return bitmap;
}

static void send_sack_ack(void) {
    uint8_t bitmap = generate_sack_bitmap();
    
    // Send ACK frame with byte stuffing
    uart_send_byte(FRAME_BYTE);
    send_escaped_byte(ACK);
    send_escaped_byte((expected_seq >> 8) & 0xFF);
    send_escaped_byte(expected_seq & 0xFF);
    send_escaped_byte(bitmap);
    uart_send_byte(FRAME_BYTE);
    
    conn_state.last_sent_ack_time_ms = millis();
}

static void send_nack(void) {
    // Send NACK frame with byte stuffing
    uart_send_byte(FRAME_BYTE);
    send_escaped_byte(NACK);
    uart_send_byte(FRAME_BYTE);
}

static void process_in_order_packets(uint8_t *output, uint16_t *output_len) {
    while (recv_window[0].received && recv_window[0].pkt.seq == expected_seq) {
        Packet *p = &recv_window[0].pkt;
        memcpy(&output[*output_len], p->data, p->data_length);
        *output_len += p->data_length;

        if (p->flags & FLAG_LAST_PACKET) {
            break;
        }

        for (int i = 1; i < WINDOW_SIZE; i++) {
            recv_window[i - 1] = recv_window[i];
        }
        recv_window[WINDOW_SIZE - 1].received = false;
        expected_seq = (expected_seq + 1) % MAX_SEQ_NUM;
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

static void prepare_and_send_packet(const uint8_t *data, uint16_t len, uint8_t flags, uint16_t total_packets, uint32_t total_size, uint8_t encoding_type) {
    uint8_t index = next_seq % WINDOW_SIZE;

    Packet *pkt = &send_window[index].pkt;
    pkt->flags = flags;
    pkt->seq = next_seq;
    pkt->seq_length = total_packets;
    pkt->seq_size = total_size;
    pkt->data_length = len;
    pkt->encoding_type = encoding_type;
    memcpy(pkt->data, data, len);
    pkt->token = SHARED_TOKEN;

    send_packet_struct(pkt);

    send_window[index].sent = true;
    send_window[index].acked = false;
    send_window[index].last_sent_time_ms = millis();
    send_window[index].retransmit_count = 0;

    next_seq = (next_seq + 1) % MAX_SEQ_NUM;
    
    uint8_t current_window_usage = ((next_seq - base_seq + MAX_SEQ_NUM) % MAX_SEQ_NUM);
    if (current_window_usage > conn_state.max_window_usage) {
        conn_state.max_window_usage = current_window_usage;
    }
}

// Public wrapper: send a single-packet payload directly through the sliding-window
// This constructs a single packet and sends it through the existing sender logic.
void sliding_window_send_payload(const uint8_t *data, uint16_t len, uint8_t flags, uint8_t encoding_type) {
    // For a single packet, total_packets = 1 and total_size = len
    prepare_and_send_packet(data, len, flags, 1, len, encoding_type);
}

static void handle_sack_ack(uint16_t sack_base, uint8_t bitmap) {
    bool valid_ack = false;
    
    for (int i = 0; i < WINDOW_SIZE; i++) {
        uint16_t seq = (sack_base + i) % MAX_SEQ_NUM;
        uint8_t index = seq % WINDOW_SIZE;

        if (bitmap & (1 << i)) {
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

    while (send_window[base_seq % WINDOW_SIZE].acked) {
        send_window[base_seq % WINDOW_SIZE].sent = false;
        send_window[base_seq % WINDOW_SIZE].acked = false;
        send_window[base_seq % WINDOW_SIZE].retransmit_count = 0;
        base_seq = (base_seq + 1) % MAX_SEQ_NUM;
    }
}

static void check_timeouts_and_retransmit(void) {
    uint32_t now = millis();
    
    for (int i = 0; i < WINDOW_SIZE; i++) {
        if (send_window[i].sent && !send_window[i].acked) {
            uint32_t timeout = TIMEOUT_MS * (1 << send_window[i].retransmit_count);
            
            if ((now - send_window[i].last_sent_time_ms) > timeout) {
                if (send_window[i].retransmit_count < MAX_RETRANSMIT_COUNT) {
                    send_packet_struct(&send_window[i].pkt);
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
    
    base_seq = 0;
    next_seq = 0;
    expected_seq = 0;
    
    for (int i = 0; i < WINDOW_SIZE; i++) {
        recv_window[i].received = false;
    }
    
    conn_state.resets_received++;
    conn_state.connection_active = false;
}

static bool validate_packet_sequence(Packet *pkt) {
    uint16_t seq_diff = (pkt->seq - expected_seq + MAX_SEQ_NUM) % MAX_SEQ_NUM;
    
    if (seq_diff >= WINDOW_SIZE) {
        conn_state.sequence_mismatch_drop++;
        return false;
    }
    
    return true;
}
