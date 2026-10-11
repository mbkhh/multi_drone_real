#!/usr/bin/env python3

import serial
import struct
import argparse
import time
import threading


MAGIC = b'\xAA\x55'

# Application IDs
STATION = 100
LEADER = 1
BROADCAST = 255


# frame:
#
# MAGIC
# source uint8
# destination uint8
# seq uint32
# payload_len uint16
# payload
#

HEADER = "!2sBBIH"


def build_frame(src, dst, seq, payload):

    return struct.pack(
        HEADER,
        MAGIC,
        src,
        dst,
        seq,
        len(payload)
    ) + payload



def parse_frame(data):

    if len(data) < 10:
        return None

    magic, src, dst, seq, length = struct.unpack(
        HEADER,
        data[:10]
    )

    if magic != MAGIC:
        return None

    if len(data) < 10 + length:
        return None

    payload = data[10:10+length]

    return {
        "src": src,
        "dst": dst,
        "seq": seq,
        "payload": payload
    }



class P900Test:

    def __init__(self, port, role):

        self.ser = serial.Serial(
            port,
            230400,
            timeout=0.05
        )

        self.role = role

        if role == "station":
            self.id = STATION

        elif role == "leader":
            self.id = LEADER

        elif role == "follower":
            self.id = 2

        self.seq = 0

        self.rx_buffer = b""


        print()
        print("======================")
        print("P900 PMP TEST")
        print("======================")
        print("role:", role)
        print("id:", self.id)
        print("port:", port)
        print("======================")
        print()


    def send(self, dst, text):

        self.seq += 1

        payload = text.encode()

        frame = build_frame(
            self.id,
            dst,
            self.seq,
            payload
        )

        self.ser.write(frame)

        print(
            f"TX src={self.id} dst={dst} "
            f"seq={self.seq} msg={text}"
        )



    def rx_loop(self):

        while True:

            data = self.ser.read(256)

            if data:
                self.rx_buffer += data


            while True:

                idx = self.rx_buffer.find(MAGIC)

                if idx < 0:
                    self.rx_buffer = b""
                    break


                if idx > 0:
                    self.rx_buffer = self.rx_buffer[idx:]


                if len(self.rx_buffer) < 10:
                    break


                frame = parse_frame(self.rx_buffer)

                if frame is None:
                    break


                self.rx_buffer = self.rx_buffer[
                    10 + len(frame["payload"]):
                ]


                self.handle_rx(frame)



    def handle_rx(self, frame):

        src = frame["src"]
        dst = frame["dst"]
        seq = frame["seq"]
        msg = frame["payload"].decode(errors="ignore")


        # application filtering

        if dst != self.id and dst != BROADCAST:
            print(
                f"DROP packet for {dst}"
            )
            return


        print(
            f"RX src={src} dst={dst} "
            f"seq={seq} msg={msg}"
        )



    def run(self):

        threading.Thread(
            target=self.rx_loop,
            daemon=True
        ).start()


        while True:

            cmd = input("> ")

            if cmd == "quit":
                break


            if self.role == "station":

                # station commands leader

                self.send(
                    LEADER,
                    cmd
                )


            elif self.role == "leader":

                if cmd.startswith("f"):

                    # example:
                    # f hello

                    self.send(
                        BROADCAST,
                        cmd[2:]
                    )

                elif cmd.startswith("s"):

                    self.send(
                        STATION,
                        cmd[2:]
                    )


            elif self.role == "follower":

                # send everything to leader

                self.send(
                    LEADER,
                    cmd
                )



if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--port",
        required=True
    )

    parser.add_argument(
        "--role",
        required=True,
        choices=[
            "station",
            "leader",
            "follower"
        ]
    )


    args = parser.parse_args()


    app = P900Test(
        args.port,
        args.role
    )

    app.run()
