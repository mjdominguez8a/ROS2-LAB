"""Share the SO-100's USB serial port with the dev container (macOS / Docker Desktop).

Docker Desktop on a Mac cannot pass USB devices into containers, so this script runs on the
Mac itself and exposes the robot's serial port on TCP 127.0.0.1:7001. Inside the container,
`bash scripts/connect_robot.sh` turns that back into /dev/so100.

    uv run --with pyserial scripts/mac_serial_bridge.py               # auto-detect the robot
    uv run --with pyserial scripts/mac_serial_bridge.py --port /dev/cu.usbmodemXXXX

Leave it running while you use the robot; Ctrl+C to stop. Only this Mac can connect.
"""
import argparse
import selectors
import socket
import sys

import serial
from serial.tools import list_ports

BAUD = 1_000_000
WCH_VID = 0x1A86  # CH340 / CH343 USB-serial chips on the Waveshare/Feetech boards


def find_port():
    for p in list_ports.comports():
        if p.vid == WCH_VID and p.device.startswith('/dev/cu.'):
            return p.device
    return None


def serve(srv, ser):
    """Copy bytes between the serial port and the most recent TCP client, forever.

    A new connection replaces the current one: when the container side goes away, Docker Desktop
    can keep the Mac side of its connection open, and that stale client must not block the next one.
    """
    sel = selectors.DefaultSelector()
    sel.register(srv, selectors.EVENT_READ, 'listen')
    sel.register(ser.fileno(), selectors.EVENT_READ, 'serial')
    conn = None

    def drop(reason):
        nonlocal conn
        sel.unregister(conn)
        conn.close()
        conn = None
        print(reason)

    while True:
        for key, _ in sel.select():
            if key.data == 'listen':
                new, addr = srv.accept()
                new.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                if conn is not None:
                    drop('Previous connection replaced')
                conn = new
                sel.register(conn, selectors.EVENT_READ, 'tcp')
                ser.reset_input_buffer()
                print(f'Container connected ({addr[0]}:{addr[1]})')
            elif key.data == 'tcp':
                if key.fileobj is not conn:
                    continue  # replaced earlier in this same batch of events
                try:
                    data = conn.recv(4096)
                except OSError as e:
                    data = None
                    print(f'Connection error: {e}')
                if data:
                    ser.write(data)
                else:
                    drop('Container disconnected; waiting for a new connection')
            else:
                data = ser.read(ser.in_waiting or 1)
                if data and conn is not None:
                    try:
                        conn.sendall(data)
                    except OSError as e:
                        drop(f'Connection error: {e}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--port', help='serial device (default: auto-detect /dev/cu.* with a WCH chip)')
    ap.add_argument('--tcp', type=int, default=7001, help='TCP port to listen on (default 7001)')
    args = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)  # show messages immediately, even when piped

    port = args.port or find_port()
    if not port:
        print('Robot not found. Is the USB cable plugged in? Ports seen:')
        for p in list_ports.comports():
            print(f'  {p.device}  {p.description}  vid={p.vid}')
        return 1

    ser = serial.Serial(port, BAUD, timeout=0)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', args.tcp))  # localhost only: nobody on the network can drive the arm
    srv.listen(4)
    print(f'Robot serial port: {port} @ {BAUD} baud')
    print(f'Listening on 127.0.0.1:{args.tcp}. In the container run: bash scripts/connect_robot.sh')
    print('Ctrl+C to stop.')
    try:
        serve(srv, ser)
    except KeyboardInterrupt:
        print('\nStopping.')
    finally:
        srv.close()
        ser.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
