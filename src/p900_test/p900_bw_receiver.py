#!/usr/bin/env python3

import argparse
import serial
import struct
import time
import zlib


MAGIC = b"P900"

# magic(4) + seq(4) + payload_size(2)
HEADER = struct.Struct("<4sIH")
CRC = struct.Struct("<I")

MAX_PAYLOAD = 4096


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=57600)
    args = parser.parse_args()

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0.1,
    )

    ser.reset_input_buffer()

    print(f"Listening on {args.port} @ {args.baud}")
    print("Waiting for bandwidth stream...\n")

    buffer = bytearray()

    total_packets = 0
    total_payload_bytes = 0
    total_frame_bytes = 0

    interval_packets = 0
    interval_payload_bytes = 0
    interval_frame_bytes = 0

    crc_errors = 0
    malformed = 0
    missing_packets = 0

    expected_seq = None

    start_time = time.monotonic()
    interval_start = start_time

    try:
        while True:

            data = ser.read(4096)

            if data:
                buffer.extend(data)

            while True:

                # Need at least a header
                if len(buffer) < HEADER.size:
                    break

                # Find frame synchronization marker
                magic_pos = buffer.find(MAGIC)

                if magic_pos == -1:
                    # Keep last few bytes in case MAGIC spans reads
                    if len(buffer) > len(MAGIC):
                        del buffer[:-len(MAGIC)]
                    break

                if magic_pos > 0:
                    del buffer[:magic_pos]

                if len(buffer) < HEADER.size:
                    break

                magic, seq, payload_size = HEADER.unpack(
                    buffer[:HEADER.size]
                )

                if payload_size > MAX_PAYLOAD:
                    malformed += 1
                    del buffer[0]
                    continue

                frame_size = HEADER.size + payload_size + CRC.size

                if len(buffer) < frame_size:
                    break

                frame = bytes(buffer[:frame_size])
                del buffer[:frame_size]

                payload_start = HEADER.size
                payload_end = payload_start + payload_size

                payload = frame[payload_start:payload_end]

                received_crc = CRC.unpack(
                    frame[payload_end:payload_end + CRC.size]
                )[0]

                calculated_crc = zlib.crc32(
                    frame[4:payload_end]
                ) & 0xFFFFFFFF

                if received_crc != calculated_crc:
                    crc_errors += 1
                    continue

                # Packet-loss calculation
                if expected_seq is not None:
                    if seq > expected_seq:
                        missing_packets += seq - expected_seq

                expected_seq = seq + 1

                total_packets += 1
                total_payload_bytes += payload_size
                total_frame_bytes += frame_size

                interval_packets += 1
                interval_payload_bytes += payload_size
                interval_frame_bytes += frame_size

            now = time.monotonic()

            if now - interval_start >= 1.0:

                elapsed = now - interval_start

                payload_kbps = (
                    interval_payload_bytes * 8
                    / elapsed
                    / 1000
                )

                frame_kbps = (
                    interval_frame_bytes * 8
                    / elapsed
                    / 1000
                )

                print(
                    f"payload={payload_kbps:7.2f} kbit/s | "
                    f"wire={frame_kbps:7.2f} kbit/s | "
                    f"pps={interval_packets / elapsed:6.1f} | "
                    f"packets={total_packets} | "
                    f"missing={missing_packets} | "
                    f"crc={crc_errors}"
                )

                interval_packets = 0
                interval_payload_bytes = 0
                interval_frame_bytes = 0
                interval_start = now

    except KeyboardInterrupt:

        total_time = time.monotonic() - start_time

        print("\n--- FINAL RESULTS ---")

        if total_time > 0:

            avg_payload_kbps = (
                total_payload_bytes * 8
                / total_time
                / 1000
            )

            print(f"Duration:       {total_time:.2f} s")
            print(f"Good packets:   {total_packets}")
            print(f"Missing packets:{missing_packets}")
            print(f"CRC errors:     {crc_errors}")
            print(f"Malformed:      {malformed}")
            print(
                f"Average payload: {avg_payload_kbps:.2f} kbit/s"
            )

    finally:
        ser.close()


if __name__ == "__main__":
    main()
