#!/usr/bin/env python3

import argparse
import os
import serial
import struct
import time
import zlib


MAGIC = b"P900"

HEADER = struct.Struct("<4sIH")
CRC = struct.Struct("<I")


def make_packet(seq, payload):

    header = HEADER.pack(
        MAGIC,
        seq,
        len(payload)
    )

    # CRC over seq + size + payload
    crc_value = zlib.crc32(
        header[4:] + payload
    ) & 0xFFFFFFFF

    return (
        header
        + payload
        + CRC.pack(crc_value)
    )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=57600)

    parser.add_argument(
        "--payload",
        type=int,
        default=256,
        help="Payload size in bytes"
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=20,
        help="Test duration in seconds"
    )

    parser.add_argument(
        "--rate-kbps",
        type=float,
        default=0,
        help="Target payload rate. 0 = maximum possible"
    )

    args = parser.parse_args()

    if args.payload < 1 or args.payload > 4096:
        raise ValueError("payload must be 1..4096 bytes")

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=1,
        write_timeout=5,
    )

    ser.reset_output_buffer()

    #
    # Use random data rather than repeated zeros.
    #
    payload = os.urandom(args.payload)

    seq = 0

    total_payload_bytes = 0
    total_frame_bytes = 0

    interval_payload_bytes = 0
    interval_frame_bytes = 0
    interval_packets = 0

    start = time.monotonic()
    interval_start = start

    print(
        f"P900 bandwidth sender\n"
        f"Port:       {args.port}\n"
        f"Baud:       {args.baud}\n"
        f"Payload:    {args.payload} bytes\n"
        f"Duration:   {args.duration} s\n"
    )

    if args.rate_kbps == 0:
        print("Mode: MAXIMUM SPEED\n")
    else:
        print(
            f"Target payload rate: "
            f"{args.rate_kbps:.1f} kbit/s\n"
        )

    try:

        while True:

            now = time.monotonic()

            if now - start >= args.duration:
                break

            packet = make_packet(seq, payload)

            ser.write(packet)

            total_payload_bytes += len(payload)
            total_frame_bytes += len(packet)

            interval_payload_bytes += len(payload)
            interval_frame_bytes += len(packet)
            interval_packets += 1

            seq += 1

            #
            # Optional rate limiter
            #
            if args.rate_kbps > 0:

                target_bytes_per_sec = (
                    args.rate_kbps * 1000 / 8
                )

                expected_time = (
                    total_payload_bytes
                    / target_bytes_per_sec
                )

                actual_time = (
                    time.monotonic() - start
                )

                sleep_time = expected_time - actual_time

                if sleep_time > 0:
                    time.sleep(sleep_time)

            now = time.monotonic()

            if now - interval_start >= 1.0:

                elapsed = now - interval_start

                payload_kbps = (
                    interval_payload_bytes
                    * 8
                    / elapsed
                    / 1000
                )

                wire_kbps = (
                    interval_frame_bytes
                    * 8
                    / elapsed
                    / 1000
                )

                print(
                    f"TX payload={payload_kbps:7.2f} kbit/s | "
                    f"wire={wire_kbps:7.2f} kbit/s | "
                    f"pps={interval_packets / elapsed:6.1f}"
                )

                interval_payload_bytes = 0
                interval_frame_bytes = 0
                interval_packets = 0

                interval_start = now

    except KeyboardInterrupt:
        pass

    finally:

        ser.flush()

        total_time = time.monotonic() - start

        payload_kbps = (
            total_payload_bytes
            * 8
            / total_time
            / 1000
        )

        wire_kbps = (
            total_frame_bytes
            * 8
            / total_time
            / 1000
        )

        print("\n--- SENDER RESULTS ---")

        print(f"Duration:        {total_time:.2f} s")
        print(f"Packets sent:    {seq}")
        print(
            f"Payload data:    "
            f"{total_payload_bytes / 1e6:.3f} MB"
        )

        print(
            f"Payload rate:    "
            f"{payload_kbps:.2f} kbit/s"
        )

        print(
            f"Frame rate:      "
            f"{wire_kbps:.2f} kbit/s"
        )

        ser.close()


if __name__ == "__main__":
    main()
