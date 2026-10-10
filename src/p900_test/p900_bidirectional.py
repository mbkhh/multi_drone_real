#!/usr/bin/env python3

import argparse
import os
import serial
import struct
import threading
import time
import zlib


# ---------------------------------------------------------
# Packet format
#
# MAGIC        4 bytes
# NODE_ID      1 byte
# SEQUENCE     4 bytes
# PAYLOAD_LEN  2 bytes
# PAYLOAD      N bytes
# CRC32        4 bytes
#
# With payload=240:
#
# 4 + 1 + 4 + 2 + 240 + 4 = 255 bytes
#
# This fits nicely inside your P900 S112=256 setting.
# ---------------------------------------------------------

MAGIC = b"P9BD"

HEADER = struct.Struct("<4sBIH")
CRC = struct.Struct("<I")

MAX_PAYLOAD = 4096


class Stats:
    def __init__(self):
        self.lock = threading.Lock()

        self.tx_bytes_interval = 0
        self.tx_packets_interval = 0

        self.rx_bytes_interval = 0
        self.rx_packets_interval = 0

        self.tx_bytes_total = 0
        self.tx_packets_total = 0

        self.rx_bytes_total = 0
        self.rx_packets_total = 0

        self.crc_errors = 0
        self.malformed = 0
        self.missing = 0
        self.duplicates = 0

        self.expected_seq = {}


def make_packet(node_id, seq, payload):

    header = HEADER.pack(
        MAGIC,
        node_id,
        seq,
        len(payload)
    )

    crc_value = zlib.crc32(
        header[4:] + payload
    ) & 0xFFFFFFFF

    return header + payload + CRC.pack(crc_value)


def receiver_thread(ser, stats, stop_event):

    buffer = bytearray()

    while not stop_event.is_set():

        try:
            data = ser.read(4096)
        except serial.SerialException as e:
            print(f"\nSerial RX error: {e}")
            stop_event.set()
            return

        if data:
            buffer.extend(data)

        while True:

            if len(buffer) < HEADER.size:
                break

            # Find synchronization marker
            pos = buffer.find(MAGIC)

            if pos == -1:

                # Keep final bytes in case MAGIC is split
                # across two serial reads
                keep = len(MAGIC) - 1

                if len(buffer) > keep:
                    del buffer[:-keep]

                break

            if pos > 0:
                del buffer[:pos]

            if len(buffer) < HEADER.size:
                break

            try:
                magic, source_node, seq, payload_size = HEADER.unpack(
                    buffer[:HEADER.size]
                )
            except struct.error:
                break

            if payload_size > MAX_PAYLOAD:

                with stats.lock:
                    stats.malformed += 1

                del buffer[0]
                continue

            frame_size = (
                HEADER.size
                + payload_size
                + CRC.size
            )

            if len(buffer) < frame_size:
                break

            frame = bytes(buffer[:frame_size])

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

                with stats.lock:
                    stats.crc_errors += 1

                # Delete only one byte and search for MAGIC again.
                # This gives us better recovery if bytes were dropped.
                del buffer[0]
                continue

            # Valid frame
            del buffer[:frame_size]

            with stats.lock:

                expected = stats.expected_seq.get(source_node)

                if expected is not None:

                    if seq > expected:
                        stats.missing += seq - expected

                    elif seq < expected:
                        stats.duplicates += 1

                stats.expected_seq[source_node] = seq + 1

                stats.rx_bytes_interval += payload_size
                stats.rx_packets_interval += 1

                stats.rx_bytes_total += payload_size
                stats.rx_packets_total += 1


def transmitter_thread(
    ser,
    node_id,
    payload_size,
    target_rate_kbps,
    stats,
    stop_event
):

    payload = os.urandom(payload_size)

    seq = 0

    start_time = time.monotonic()
    payload_sent = 0

    target_bytes_per_second = (
        target_rate_kbps * 1000.0 / 8.0
    )

    while not stop_event.is_set():

        packet = make_packet(
            node_id,
            seq,
            payload
        )

        try:
            ser.write(packet)

        except serial.SerialTimeoutException:
            print("\nSerial TX timeout")
            stop_event.set()
            return

        except serial.SerialException as e:
            print(f"\nSerial TX error: {e}")
            stop_event.set()
            return

        with stats.lock:

            stats.tx_bytes_interval += payload_size
            stats.tx_packets_interval += 1

            stats.tx_bytes_total += payload_size
            stats.tx_packets_total += 1

        payload_sent += payload_size

        seq += 1

        # --------------------------------------------------
        # Rate limiter
        #
        # Calculate where we SHOULD be in time rather
        # than just sleeping a fixed amount every packet.
        # --------------------------------------------------

        expected_elapsed = (
            payload_sent / target_bytes_per_second
        )

        actual_elapsed = (
            time.monotonic() - start_time
        )

        sleep_time = (
            expected_elapsed - actual_elapsed
        )

        if sleep_time > 0:
            time.sleep(sleep_time)


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--port",
        required=True
    )

    parser.add_argument(
        "--node",
        type=int,
        required=True,
        help="Node ID, e.g. 1 for ground, 2 for drone"
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=230400
    )

    parser.add_argument(
        "--payload",
        type=int,
        default=240
    )

    parser.add_argument(
        "--tx-rate-kbps",
        type=float,
        default=20.0,
        help="Payload rate transmitted by THIS node"
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=60.0
    )

    args = parser.parse_args()

    if not (0 <= args.node <= 255):
        raise ValueError("node must be 0..255")

    if not (1 <= args.payload <= MAX_PAYLOAD):
        raise ValueError(
            f"payload must be 1..{MAX_PAYLOAD}"
        )

    if args.tx_rate_kbps <= 0:
        raise ValueError(
            "tx-rate-kbps must be greater than zero"
        )

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0.05,
        write_timeout=2.0
    )

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    stats = Stats()

    stop_event = threading.Event()

    print()
    print("P900 BIDIRECTIONAL TEST")
    print("-----------------------")
    print(f"Node ID:       {args.node}")
    print(f"Port:          {args.port}")
    print(f"Baud:          {args.baud}")
    print(f"Payload:       {args.payload} bytes")
    print(f"TX target:     {args.tx_rate_kbps:.1f} kbit/s")
    print(f"Duration:      {args.duration:.1f} s")
    print()

    rx_thread = threading.Thread(
        target=receiver_thread,
        args=(
            ser,
            stats,
            stop_event
        ),
        daemon=True
    )

    tx_thread = threading.Thread(
        target=transmitter_thread,
        args=(
            ser,
            args.node,
            args.payload,
            args.tx_rate_kbps,
            stats,
            stop_event
        ),
        daemon=True
    )

    rx_thread.start()
    tx_thread.start()

    start = time.monotonic()
    last_print = start

    try:

        while True:

            time.sleep(0.05)

            now = time.monotonic()

            if now - start >= args.duration:
                break

            if now - last_print >= 1.0:

                elapsed = now - last_print

                with stats.lock:

                    tx_kbps = (
                        stats.tx_bytes_interval
                        * 8
                        / elapsed
                        / 1000
                    )

                    rx_kbps = (
                        stats.rx_bytes_interval
                        * 8
                        / elapsed
                        / 1000
                    )

                    tx_pps = (
                        stats.tx_packets_interval
                        / elapsed
                    )

                    rx_pps = (
                        stats.rx_packets_interval
                        / elapsed
                    )

                    print(
                        f"TX={tx_kbps:7.2f} kbps "
                        f"({tx_pps:5.1f} pps) | "
                        f"RX={rx_kbps:7.2f} kbps "
                        f"({rx_pps:5.1f} pps) | "
                        f"missing={stats.missing} | "
                        f"crc={stats.crc_errors} | "
                        f"dup={stats.duplicates}"
                    )

                    stats.tx_bytes_interval = 0
                    stats.tx_packets_interval = 0

                    stats.rx_bytes_interval = 0
                    stats.rx_packets_interval = 0

                last_print = now

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:

        stop_event.set()

        tx_thread.join(timeout=2)
        rx_thread.join(timeout=2)

        elapsed = time.monotonic() - start

        with stats.lock:

            avg_tx = (
                stats.tx_bytes_total
                * 8
                / elapsed
                / 1000
            )

            avg_rx = (
                stats.rx_bytes_total
                * 8
                / elapsed
                / 1000
            )

            print()
            print("--- FINAL RESULTS ---")
            print(f"Duration:         {elapsed:.2f} s")
            print(f"TX packets:       {stats.tx_packets_total}")
            print(f"RX packets:       {stats.rx_packets_total}")
            print(f"TX payload rate:  {avg_tx:.2f} kbit/s")
            print(f"RX payload rate:  {avg_rx:.2f} kbit/s")
            print(f"Missing packets:  {stats.missing}")
            print(f"CRC errors:       {stats.crc_errors}")
            print(f"Duplicates:       {stats.duplicates}")
            print(f"Malformed:        {stats.malformed}")

        ser.close()


if __name__ == "__main__":
    main()
