#!/usr/bin/env python3

import argparse
import time
import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True,
                        help="Serial port, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Seconds between packets")
    parser.add_argument("--timeout", type=float, default=2.0,
                        help="ACK timeout in seconds")
    args = parser.parse_args()

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=args.timeout,
    )

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print(f"P900 sender using {args.port} at {args.baud} baud")
    print("Press Ctrl+C to stop.\n")

    seq = 0
    sent = 0
    received = 0

    try:
        while True:
            seq += 1
            sent += 1

            send_time_ns = time.time_ns()

            packet = f"PING,{seq},{send_time_ns}\n"

            ser.write(packet.encode("ascii"))
            ser.flush()

            print(f"TX  seq={seq}")

            response = ser.readline()

            receive_time_ns = time.time_ns()

            if not response:
                print(f"TIMEOUT seq={seq}")

            else:
                try:
                    message = response.decode("ascii").strip()
                except UnicodeDecodeError:
                    print(f"RX invalid bytes: {response!r}")
                    time.sleep(args.interval)
                    continue

                parts = message.split(",")

                if len(parts) == 3 and parts[0] == "ACK":
                    ack_seq = int(parts[1])

                    if ack_seq == seq:
                        received += 1

                        rtt_ms = (
                            receive_time_ns - send_time_ns
                        ) / 1_000_000.0

                        loss_percent = (
                            (sent - received) / sent
                        ) * 100.0

                        print(
                            f"RX  ACK seq={ack_seq}  "
                            f"RTT={rtt_ms:.2f} ms  "
                            f"loss={loss_percent:.1f}%"
                        )

                    else:
                        print(
                            f"Unexpected ACK: expected {seq}, "
                            f"received {ack_seq}"
                        )

                else:
                    print(f"Unexpected RX: {message}")

            print()

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nStopping sender.")

        if sent > 0:
            loss = ((sent - received) / sent) * 100.0

            print(f"Sent:     {sent}")
            print(f"Received: {received}")
            print(f"Lost:     {sent - received}")
            print(f"Loss:     {loss:.2f}%")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
