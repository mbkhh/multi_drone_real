#!/usr/bin/env python3

import argparse
import time
import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True,
                        help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=57600)
    args = parser.parse_args()

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1.0,
    )

    # Clear anything left in buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print(f"P900 receiver listening on {args.port} at {args.baud} baud")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            line = ser.readline()

            if not line:
                continue

            try:
                message = line.decode("ascii").strip()
            except UnicodeDecodeError:
                print(f"RX invalid bytes: {line!r}")
                continue

            receive_time_ns = time.time_ns()

            print(f"RX: {message}")

            parts = message.split(",")

            if len(parts) != 3:
                print("Malformed packet")
                continue

            msg_type, seq_str, sender_time_str = parts

            if msg_type != "PING":
                print(f"Unknown message type: {msg_type}")
                continue

            try:
                seq = int(seq_str)
                sender_time_ns = int(sender_time_str)
            except ValueError:
                print("Malformed packet values")
                continue

            # Send acknowledgement back to ground station
            ack = f"ACK,{seq},{receive_time_ns}\n"

            ser.write(ack.encode("ascii"))
            ser.flush()

            print(f"TX: ACK seq={seq}")

    except KeyboardInterrupt:
        print("\nStopping receiver.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()

