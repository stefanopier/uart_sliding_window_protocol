#!/usr/bin/env python3
"""
Sliding Window Protocol Test Client
Implements the 16-bit sequence number sliding window protocol with CRC-16, SACK, and byte stuffing.
Compatible with the MCU implementation in sliding_window_protocol_16bit.c
"""

import serial
import struct
import time
import sys
import argparse
import json
import math
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import Enum

# Protocol Constants
WINDOW_SIZE = 8
MAX_SEQ_NUM = 65536
MAX_PAYLOAD = 1024  # Total per-frame budget (matches MCU ring buffer)
FRAME_OVERHEAD = 12 + 2 + 2  # header + token + CRC bytes after unstuffing
MAX_FRAME_SIZE = MAX_PAYLOAD  # Alias for readability when discussing budgets

TIMEOUT_MS = 100
CONNECTION_TIMEOUT_MS = 5000
SHARED_TOKEN = 0xABCD

# Target profiles (payload caps, boot delays, and connect behavior)
TARGET_PROFILES = {
    'uno': {
        'max_payload': 44,
        'boot_delay': 1.8,
        'flush_on_connect': True,
    },
    'generic': {
        'max_payload': MAX_PAYLOAD,
        'boot_delay': 0.0,
        'flush_on_connect': False,
    },
}

# Control bytes
ACK = 0x06
NACK = 0x15
RESET_FRAME = 0x55

# Framing characters
FRAME_BYTE = 0x7E
ESCAPE_CHAR = 0x7D
STUFF_BYTE = 0x20

# Flags
FLAG_LAST_PACKET = 0x01

# Encoding types
ENCODING_BINARY = 0x00
ENCODING_ASCII = 0x01
ENCODING_UTF8 = 0x02
ENCODING_JSON = 0x03
ENCODING_CBOR = 0x04
ENCODING_BASE64 = 0x05
ENCODING_HEX = 0x06


class FrameType(Enum):
    DATA = 0
    ACK = 1
    NACK = 2
    RESET = 3
    UNKNOWN = 4


@dataclass
class Packet:
    flags: int
    seq: int
    seq_length: int
    seq_size: int
    data_length: int
    encoding_type: int
    data: bytes
    token: int
    crc: int


class SlidingWindowClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_ms: int = TIMEOUT_MS,
        conn_timeout_ms: int = CONNECTION_TIMEOUT_MS,
        max_payload: Optional[int] = MAX_PAYLOAD,
        boot_delay: float = 0.0,
        flush_on_connect: bool = False,
    ):
        """Initialize the sliding window protocol client."""
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.expected_seq = 0
        self.recv_window = [None] * WINDOW_SIZE
        # Timeouts (ms)
        self.timeout_ms = int(timeout_ms)
        self.conn_timeout_ms = int(conn_timeout_ms)
        self.max_payload = max_payload
        self.boot_delay = boot_delay
        self.flush_on_connect = flush_on_connect

        # Statistics
        self.stats = {
            'packets_sent': 0,
            'packets_received': 0,
            'acks_sent': 0,
            'nacks_sent': 0,
            'crc_errors': 0,
            'timeouts': 0
        }
        self._next_seq = 0
    
    def connect(self):
        """Open serial connection."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout_ms / 1000.0
            )
            if self.boot_delay > 0:
                time.sleep(self.boot_delay)
            if self.flush_on_connect:
                if self.serial.in_waiting:
                    self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("✓ Disconnected")
    
    @staticmethod
    def compute_crc16(data: bytes) -> int:
        """Compute CRC-16-CCITT."""
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc
    
    def send_escaped_byte(self, byte: int):
        """Send a byte with escape stuffing if needed."""
        # Deprecated: kept for compatibility, prefer send_packet building wire bytes
        if byte == FRAME_BYTE or byte == ESCAPE_CHAR:
            self.serial.write(bytes([ESCAPE_CHAR, byte ^ STUFF_BYTE]))
        else:
            self.serial.write(bytes([byte]))
    
    def send_packet(self, data: bytes, flags: int = FLAG_LAST_PACKET, 
                   encoding_type: int = ENCODING_ASCII, seq: int = 0,
                   seq_length: int = 1, seq_size: Optional[int] = None):
        """Send a data packet with framing and CRC."""
        if self.max_payload is not None and len(data) > self.max_payload:
            raise ValueError(
                f"Payload len {len(data)} exceeds configured limit ({self.max_payload} bytes)."
            )
        if seq_length < 1:
            raise ValueError("seq_length must be at least 1")
        if seq_size is None:
            seq_size = len(data)
        # Build packet structure: flags(1) + seq(2) + seq_length(2) + seq_size(4) + data_length(2) + encoding_type(1)
        packet_data = struct.pack(
            '<BHHLHB',
            flags,
            seq,
            seq_length,
            seq_size,
            len(data),  # data_length (2 bytes)
            encoding_type  # encoding_type (1 byte)
        )
        packet_data += data
        packet_data += struct.pack('<H', SHARED_TOKEN)
        
        # Compute CRC over everything except CRC field
        crc = self.compute_crc16(packet_data)
        packet_data += struct.pack('<H', crc)
        
        # Build wire bytes with byte-stuffing so we can log the exact bytes
        wire = bytearray()
        wire.append(FRAME_BYTE)
        for byte in packet_data:
            if byte == FRAME_BYTE or byte == ESCAPE_CHAR:
                wire.append(ESCAPE_CHAR)
                wire.append(byte ^ STUFF_BYTE)
            else:
                wire.append(byte)
        wire.append(FRAME_BYTE)

        # Send once
        self.serial.write(bytes(wire))
        self.serial.flush()

        if getattr(self, 'debug', False):
            print(f"[DBG RAW TX] {wire.hex()}")
        
        self.stats['packets_sent'] += 1
        preview = data[:32].decode('ascii', errors='replace')
        if len(data) > 32:
            preview += '…'
        seq_desc = f"{seq}/{seq_length}" if seq_length > 1 else str(seq)
        print(f"→ Sent packet seq={seq_desc}, len={len(data)}, frame_bytes={len(wire)}, preview='{preview}'")

    def _send_and_wait(self, data: bytes, encoding_type: int):
        """Send a payload and wait for a response frame."""
        try:
            self.send_packet(data, flags=FLAG_LAST_PACKET, encoding_type=encoding_type, seq=0)
        except ValueError as exc:
            print(f"✗ {exc}")
            return None

        print("  Waiting for echo response...")
        start_time = time.time()

        while (time.time() - start_time) < (self.conn_timeout_ms / 1000.0):
            frame_type, packet, ctrl_data = self.receive_frame()

            if frame_type == FrameType.DATA and packet:
                self.send_ack(packet.seq, bitmap=0x01)
                return packet

            if frame_type == FrameType.ACK:
                print("← Received ACK")
            elif frame_type == FrameType.NACK:
                print("← Received NACK")

        print("✗ Timeout waiting for echo")
        self.stats['timeouts'] += 1
        return None
    
    def send_ack(self, seq_base: int, bitmap: int = 0x01):
        """Send a SACK acknowledgment."""
        wire = bytearray()
        wire.append(FRAME_BYTE)
        for b in (ACK, (seq_base >> 8) & 0xFF, seq_base & 0xFF, bitmap):
            if b == FRAME_BYTE or b == ESCAPE_CHAR:
                wire.append(ESCAPE_CHAR)
                wire.append(b ^ STUFF_BYTE)
            else:
                wire.append(b)
        wire.append(FRAME_BYTE)
        self.serial.write(bytes(wire))
        self.serial.flush()

        if getattr(self, 'debug', False):
            print(f"[DBG RAW TX] {wire.hex()}")
        
        self.stats['acks_sent'] += 1
        print(f"→ Sent ACK seq_base={seq_base}, bitmap=0x{bitmap:02X}")
    
    def send_nack(self):
        """Send a NACK."""
        wire = bytes([FRAME_BYTE, NACK, FRAME_BYTE])
        self.serial.write(wire)
        self.serial.flush()
        if getattr(self, 'debug', False):
            print(f"[DBG RAW TX] {wire.hex()}")
        
        self.stats['nacks_sent'] += 1
        print(f"→ Sent NACK")
    
    def receive_byte(self) -> Optional[int]:
        """Receive a single byte with timeout."""
        byte = self.serial.read(1)
        if byte:
            val = byte[0]
            if getattr(self, 'debug', False):
                print(f"[DBG RAW RX] 0x{val:02X}")
            return val
        return None
    
    def receive_frame(self) -> Tuple[Optional[FrameType], Optional[Packet], Optional[bytes]]:
        """Receive and decode a frame."""
        # Wait for frame start
        start_time = time.time()
        while (time.time() - start_time) < (self.timeout_ms / 1000.0):
            byte = self.receive_byte()
            if byte is None:
                continue
            if byte == FRAME_BYTE:
                break
        else:
            if getattr(self, 'debug', False):
                print("[DBG] Frame start timeout")
            return None, None, None
        
        # Read frame content with unstuffing
        frame_data = bytearray()
        in_escape = False
        start_time = time.time()
        
        while (time.time() - start_time) < (self.timeout_ms / 1000.0):
            byte = self.receive_byte()
            if byte is None:
                continue
            
            if in_escape:
                frame_data.append(byte ^ STUFF_BYTE)
                in_escape = False
            elif byte == ESCAPE_CHAR:
                in_escape = True
            elif byte == FRAME_BYTE:
                break
            else:
                frame_data.append(byte)

        if getattr(self, 'debug', False):
            print(f"[DBG RAW FRAME] {frame_data.hex()}")

        if not frame_data:
            return None, None, None
        
        # Check if it's a control frame
        if frame_data[0] in [ACK, NACK, RESET_FRAME]:
            frame_type = {
                ACK: FrameType.ACK,
                NACK: FrameType.NACK,
                RESET_FRAME: FrameType.RESET
            }.get(frame_data[0], FrameType.UNKNOWN)
            if getattr(self, 'debug', False):
                print(f"[DBG] Control frame: type={frame_type.name}, len={len(frame_data)-1}, data={frame_data[1:].hex()}")
            return frame_type, None, bytes(frame_data[1:])
        
        # Parse data packet
        if len(frame_data) < 16:  # Minimum packet size
            return None, None, None
        
        try:
            header_fmt = '<BHHLHB'
            header_len = struct.calcsize(header_fmt)
            flags, seq, seq_length, seq_size, data_length, encoding_type = struct.unpack_from(
                header_fmt, frame_data, 0
            )
            
            offset = header_len
            data = bytes(frame_data[offset:offset + data_length])
            offset += data_length
            
            token, crc_received = struct.unpack_from('<HH', frame_data, offset)
            
            # Verify CRC
            crc_data = frame_data[:-2]
            crc_computed = self.compute_crc16(crc_data)
            
            if crc_computed != crc_received:
                self.stats['crc_errors'] += 1
                print(f"✗ CRC error: computed=0x{crc_computed:04X}, received=0x{crc_received:04X}")
                self.send_nack()
                return None, None, None
            
            if token != SHARED_TOKEN:
                print(f"✗ Token mismatch: expected=0x{SHARED_TOKEN:04X}, received=0x{token:04X}")
                self.send_nack()
                return None, None, None
            
            packet = Packet(
                flags=flags,
                seq=seq,
                seq_length=seq_length,
                seq_size=seq_size,
                data_length=data_length,
                encoding_type=encoding_type,
                data=data,
                token=token,
                crc=crc_received
            )
            
            self.stats['packets_received'] += 1
            if getattr(self, 'debug', False):
                print(f"[DBG] Data frame: seq={seq}, len={data_length}, enc={encoding_type}, token=0x{token:04X}")
            return FrameType.DATA, packet, None
            
        except struct.error as e:
            print(f"✗ Parse error: {e}")
            return None, None, None
    
    def send_message(self, message: str, encoding_type: int = ENCODING_ASCII):
        """Send a text message and wait for echo response."""
        data = message.encode('utf-8')
        packet = self._send_and_wait(data, encoding_type)
        if packet is None:
            return None

        echo_text = packet.data.decode('utf-8', errors='replace')
        print(f"← Received echo: '{echo_text}'")

        if echo_text == message:
            print("✓ Echo matches!")
        else:
            print("✗ Echo mismatch!")
        return echo_text

    def send_payload(self, payload: bytes, encoding_type: int = ENCODING_BINARY):
        """Send raw payload bytes and wait for echo response."""
        packet = self._send_and_wait(payload, encoding_type)
        if packet is None:
            return None

        print(f"← Received echo ({len(packet.data)} bytes)")
        if packet.data == payload:
            print("✓ Echo matches!")
        else:
            print(f"✗ Echo mismatch! Sent {len(payload)} bytes, received {len(packet.data)} bytes.")
        return packet.data

    def send_payload_sequence(
        self,
        payload: bytes,
        encoding_type: int = ENCODING_BINARY,
        chunk_size: Optional[int] = None,
    ):
        """Send a payload that spans multiple packets."""
        total_size = len(payload)
        if total_size == 0:
            print("Payload is empty; nothing to send.")
            return b""

        if chunk_size is None or chunk_size <= 0:
            chunk_size = self.max_payload or total_size

        if self.max_payload is not None and chunk_size > self.max_payload:
            print(f"Chunk size {chunk_size} exceeds max payload {self.max_payload}; using {self.max_payload} instead.")
            chunk_size = self.max_payload

        if chunk_size <= 0:
            raise ValueError("Chunk size must be positive")

        total_packets = math.ceil(total_size / chunk_size)
        start_seq = self._next_seq
        seq = start_seq

        print(f"→ Sending sequence of {total_packets} packets ({total_size} bytes total, chunk={chunk_size})")
        for index in range(total_packets):
            start = index * chunk_size
            end = min(start + chunk_size, total_size)
            chunk = payload[start:end]
            flags = FLAG_LAST_PACKET if index == (total_packets - 1) else 0
            self.send_packet(
                chunk,
                flags=flags,
                encoding_type=encoding_type,
                seq=seq,
                seq_length=total_packets,
                seq_size=total_size,
            )
            seq = (seq + 1) % MAX_SEQ_NUM

        self._next_seq = seq

        print("  Waiting for echo sequence...")
        start_time = time.time()
        received_chunks = {}
        recv_start_seq: Optional[int] = None
        expected_packets: Optional[int] = None
        expected_size = total_size
        collected_size = 0
        last_packet_seen = False
        timed_out = True

        while (time.time() - start_time) < (self.conn_timeout_ms / 1000.0):
            frame_type, packet, ctrl_data = self.receive_frame()

            if frame_type == FrameType.DATA and packet:
                if recv_start_seq is None:
                    recv_start_seq = packet.seq

                if expected_packets is None and packet.seq_length:
                    expected_packets = packet.seq_length
                    reported_size = packet.seq_size or expected_size
                    expected_size = max(expected_size, reported_size)
                    print(f"  Remote reports sequence length {expected_packets}, total size {expected_size} bytes")
                elif packet.seq_size:
                    expected_size = max(expected_size, packet.seq_size)

                bucket = 0
                if recv_start_seq is not None:
                    bucket = ((packet.seq - recv_start_seq) + MAX_SEQ_NUM) % MAX_SEQ_NUM

                if packet.seq in received_chunks:
                    print(f"← Duplicate data packet seq={packet.seq}")
                    self.send_ack(packet.seq, bitmap=0x01)
                    continue

                if expected_packets is not None and bucket >= expected_packets:
                    print(f"← Unexpected seq {packet.seq}; outside expected window (0..{expected_packets - 1})")
                    self.send_ack(packet.seq, bitmap=0x01)
                    continue

                received_chunks[packet.seq] = packet.data
                collected_size += len(packet.data)
                if packet.flags & FLAG_LAST_PACKET:
                    last_packet_seen = True
                self.send_ack(packet.seq, bitmap=0x01)

                have_all_packets = expected_packets is not None and len(received_chunks) >= expected_packets
                have_all_bytes = collected_size >= expected_size

                if have_all_bytes and (have_all_packets or expected_packets is None or last_packet_seen):
                    timed_out = False
                    break

            elif frame_type == FrameType.ACK and ctrl_data:
                if len(ctrl_data) >= 3:
                    base = (ctrl_data[0] << 8) | ctrl_data[1]
                    bitmap = ctrl_data[2]
                    print(f"← Received ACK base={base}, bitmap=0x{bitmap:02X}")
            elif frame_type == FrameType.NACK:
                print("← Received NACK")
            elif frame_type == FrameType.RESET:
                print("← Received RESET")

        if timed_out:
            received_count = len(received_chunks)
            print(f"✗ Timeout waiting for full echo sequence ({collected_size}/{expected_size} bytes, {received_count}/{expected_packets or 'unknown'} packets received)")
            self.stats['timeouts'] += 1
            return None

        # Reassemble in sequence order relative to the first sequence number observed
        ordered_seqs = sorted(received_chunks.keys(), key=lambda s: ((s - (recv_start_seq or s)) + MAX_SEQ_NUM) % MAX_SEQ_NUM)
        reassembled = b''.join(received_chunks[seq] for seq in ordered_seqs)
        print(f"← Received sequence ({len(reassembled)} bytes across {len(ordered_seqs)} packets)")
        if reassembled == payload:
            print("✓ Echo matches!")
        else:
            print(f"✗ Echo mismatch! Sent {total_size} bytes, received {len(reassembled)} bytes.")
        return reassembled
    
    def print_stats(self):
        """Print connection statistics."""
        print("\n" + "="*60)
        print("Statistics:")
        print(f"  Packets sent:     {self.stats['packets_sent']}")
        print(f"  Packets received: {self.stats['packets_received']}")
        print(f"  ACKs sent:        {self.stats['acks_sent']}")
        print(f"  NACKs sent:       {self.stats['nacks_sent']}")
        print(f"  CRC errors:       {self.stats['crc_errors']}")
        print(f"  Timeouts:         {self.stats['timeouts']}")
        print("="*60)


def main():
    """Main test application."""
    parser = argparse.ArgumentParser(description="Sliding Window Protocol Test Client")
    parser.add_argument('--port', '-p', required=False, help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baudrate')
    parser.add_argument('--timeout-ms', type=int, default=TIMEOUT_MS, help='Per-byte/frame timeout in milliseconds')
    parser.add_argument('--conn-timeout-ms', type=int, default=CONNECTION_TIMEOUT_MS, help='Overall echo wait timeout in milliseconds')
    parser.add_argument('--encoding', '-e', choices=['binary', 'ascii', 'utf8', 'json', 'cbor'], default='utf8', help='Encoding for payload')
    parser.add_argument('--json', dest='json_payload', nargs='+', help='Send a single JSON payload and exit (accepts multiple tokens)')
    parser.add_argument('--json-file', dest='json_file', help='Send JSON loaded from a file and exit')
    parser.add_argument('--payload-file', dest='payload_file', help='Send raw payload bytes from a file and exit')
    parser.add_argument('--chunk-size', type=int, help='Override per-packet payload size when splitting sequences')
    parser.add_argument('--debug', action='store_true', help='Enable verbose frame parsing logs')
    parser.add_argument('--message', dest='message', help='Send a single text message and exit')
    parser.add_argument('--target', choices=sorted(TARGET_PROFILES.keys()), default='uno',
                        help='Target profile (defines payload cap and connect behavior)')
    parser.add_argument('--max-payload', type=int, help='Override maximum payload length (bytes)')
    parser.add_argument('--boot-delay', type=float, help='Override post-open delay before first send (seconds)')
    parser.add_argument('--flush-on-connect', dest='flush_on_connect', action='store_true',
                        help='Force flushing serial buffers after opening the port')
    parser.add_argument('--no-flush-on-connect', dest='flush_on_connect', action='store_false',
                        help='Skip flushing serial buffers after opening the port (overrides target)')
    parser.set_defaults(flush_on_connect=None)
    args = parser.parse_args()

    if not args.port:
        print("Usage: python sliding_window_client.py --port COMx [--baud 115200] [--json '{...}']")
        sys.exit(1)

    port = args.port
    baudrate = args.baud

    profile = dict(TARGET_PROFILES[args.target])
    if args.max_payload is not None:
        profile['max_payload'] = args.max_payload
    if args.boot_delay is not None:
        profile['boot_delay'] = args.boot_delay
    if args.flush_on_connect is not None:
        profile['flush_on_connect'] = args.flush_on_connect

    client = SlidingWindowClient(
        port,
        baudrate,
        timeout_ms=args.timeout_ms,
        conn_timeout_ms=args.conn_timeout_ms,
        max_payload=profile.get('max_payload', MAX_PAYLOAD),
        boot_delay=profile.get('boot_delay', 0.0),
        flush_on_connect=profile.get('flush_on_connect', False),
    )
    # Bind debug flag to instance for use in receive_frame
    client.debug = bool(getattr(args, 'debug', False))

    if not client.connect():
        sys.exit(1)
    
    try:
        # If a one-shot JSON payload was provided, send it and exit
        if args.json_file:
            try:
                with open(args.json_file, 'r', encoding='utf-8') as f:
                    text = f.read()
                # Try parse to validate; if it fails, still send raw text
                try:
                    json.loads(text)
                except Exception:
                    print("Warning: JSON file content did not parse; sending raw text.")
                enc_type = ENCODING_JSON if args.encoding in ['json'] else ENCODING_UTF8
                client.send_message(text, encoding_type=enc_type)
            except Exception as e:
                print(f"Failed to read JSON file: {e}")
                sys.exit(1)
        elif args.payload_file:
            try:
                with open(args.payload_file, 'rb') as f:
                    payload = f.read()
            except Exception as e:
                print(f"Failed to read payload file: {e}")
                sys.exit(1)

            print(f"Loaded raw payload: {len(payload)} bytes from {args.payload_file}")
            chunk_size = args.chunk_size
            use_sequence = False

            if chunk_size is not None:
                if chunk_size <= 0:
                    print("Chunk size must be positive")
                    sys.exit(1)
                if client.max_payload is not None and chunk_size > client.max_payload:
                    print(f"Chunk size {chunk_size} exceeds max payload {client.max_payload}; using {client.max_payload} instead.")
                    chunk_size = client.max_payload
                use_sequence = True
            elif client.max_payload is not None and len(payload) > client.max_payload:
                chunk_size = client.max_payload
                use_sequence = True

            if use_sequence:
                client.send_payload_sequence(payload, encoding_type=ENCODING_BINARY, chunk_size=chunk_size)
            else:
                client.send_payload(payload, encoding_type=ENCODING_BINARY)
        elif args.json_payload:
            # Join tokens in case the shell split the JSON string
            raw = ' '.join(args.json_payload)
            try:
                # Validate JSON. On Windows PowerShell users often pass single-quoted JSON;
                # try json.loads first, then fall back to replacing single quotes with double quotes.
                json_obj = json.loads(raw)
            except Exception:
                try:
                    alt = raw.replace("'", '"')
                    json_obj = json.loads(alt)
                except Exception as e:
                    print(f"Warning: JSON did not parse ({e}); sending raw text as provided.")
                    enc_type = ENCODING_JSON if args.encoding in ['json'] else ENCODING_UTF8
                    client.send_message(raw, encoding_type=enc_type)
                    return

            payload = json.dumps(json_obj)
            enc_type = ENCODING_JSON if args.encoding in ['json'] else ENCODING_UTF8
            client.send_message(payload, encoding_type=enc_type)
        elif args.message:
            enc_type = ENCODING_ASCII if args.encoding == 'ascii' else ENCODING_UTF8
            client.send_message(args.message, encoding_type=enc_type)
        else:
            # Interactive mode
            print("\n" + "="*60)
            print("Sliding Window Protocol Test Client")
            print("="*60)
            print("\nCommands:")
            print("  send <message>  - Send a message and wait for echo")
            print("  stats           - Show statistics")
            print("  quit            - Exit")
            print()

            while True:
                try:
                    cmd = input("> ").strip()

                    if not cmd:
                        continue

                    if cmd.lower() in ['quit', 'exit', 'q']:
                        break

                    elif cmd.lower() == 'stats':
                        client.print_stats()

                    elif cmd.lower().startswith('send '):
                        message = cmd[5:]
                        if message:
                            print()
                            client.send_message(message)
                            print()
                        else:
                            print("Usage: send <message>")

                    else:
                        # Default: treat as message to send
                        print()
                        client.send_message(cmd)
                        print()

                except KeyboardInterrupt:
                    print("\n\nInterrupted")
                    break
                except Exception as e:
                    print(f"Error: {e}")

    finally:
        client.print_stats()
        client.disconnect()


if __name__ == '__main__':
    main()
