#ifndef SWP_INTERNAL_H
#define SWP_INTERNAL_H

/*
 * swp_internal.h
 *
 * Internal declarations for sliding_window_protocol_16bit.c internals.
 * Used by platform extension wrappers to access protocol state and helper
 * functions that are normally file-private.
 *
 * NOT for use by application/consumer code — include your platform wrapper
 * header (or sliding_window_protocol_16bit.h) instead.
 */

#include "sliding_window_protocol_16bit.h"

// #############################################################################
// ## Internal Frame Type Enum
// #############################################################################

typedef enum {
    FRAME_TYPE_DATA = 0,
    FRAME_TYPE_ACK,
    FRAME_TYPE_NACK,
    FRAME_TYPE_RESET,
    FRAME_TYPE_UNKNOWN
} FrameType;

// #############################################################################
// ## Internal State (defined in sliding_window_protocol_16bit.c)
// #############################################################################

extern BufferSlot recv_window[WINDOW_SIZE];
extern uint16_t expected_frame_index;
extern EnhancedSendSlot send_window[WINDOW_SIZE];
extern uint16_t base_frame_index;
extern uint16_t next_frame_index;
extern ConnectionState conn_state;

// #############################################################################
// ## Internal Helper Functions (defined in sliding_window_protocol_16bit.c)
// #############################################################################

// CRC-16-CCITT single-byte update (poly 0x1021)
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

// CRC-16 full computation over a buffer
uint16_t compute_crc16(const uint8_t *data, uint16_t len);

// Byte stuffing — send one byte, escaping if necessary
void send_escaped_byte(uint8_t byte);

// Send a complete frame on the wire (weak — override for TX pump, etc.)
void send_frame_struct(Frame *frame);

// Frame receive with configurable timeouts
bool receive_any_frame_timed(FrameType *type, Frame *frame,
                             uint8_t *ctrl_data, uint8_t *ctrl_len,
                             uint32_t frame_start_timeout_ms, uint32_t byte_timeout_ms);

// Store a received frame in the reorder window
void store_frame(Frame *frame);

// Send SACK (selective ACK) frame
void send_sack_ack(void);

// Send NACK frame
void send_nack(void);

// Process SACK acknowledgement
void handle_sack_ack(uint16_t sack_base, uint8_t window_frames);

// Immediately retransmit all unacked frames (NACK response)
void handle_nack_retransmit(void);

// Reset all transport state (send window, recv window, indices)
void handle_reset_frame(void);

// Check for timed-out frames and retransmit
void check_timeouts_and_retransmit(void);

// #############################################################################
// ## Weak Hooks (override in platform wrappers)
// #############################################################################

// Called at the end of handle_reset_frame().
// Override to clear platform-specific state (e.g., QSPI drain buffers).
void swp_hook_on_reset(void);

// Called after send_nack() sends the NACK frame.
// Override for post-NACK RX drain (e.g., TX pump).
void swp_hook_post_nack(void);

#endif /* SWP_INTERNAL_H */
