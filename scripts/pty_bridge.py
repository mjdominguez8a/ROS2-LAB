"""Container side of the MacBook serial bridge: TCP (Mac) <-> pseudo-terminal at /dev/so100.

Started as root by `bash scripts/connect_robot.sh`; not meant to be run by hand.

Why not socat: the ROS driver locks its serial port for exclusive use (TIOCEXCL). On a real USB
port the lock ends when the driver closes the port, but a pseudo-terminal stays alive while the
bridge holds it, so the lock would outlive the driver and every later program (calibrate, the next
launch) would fail with "Device or resource busy". This bridge keeps its own handle on the port
and clears a leftover lock as soon as no other program has the port open.
"""
import os
import selectors
import signal
import socket
import sys
import termios
import tty

LINK = '/dev/so100'
TIOCGEXCL = 0x80045440  # Linux: read the exclusive-lock flag


def in_use_by_others(pts):
    me = os.getpid()
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == me:
            continue
        try:
            fds = os.listdir(f'/proc/{pid}/fd')
            if any(os.readlink(f'/proc/{pid}/fd/{fd}') == pts for fd in fds):
                return True
        except OSError:
            pass  # process ended or not ours to inspect
    return False


def clear_stale_lock(slave, pts):
    import fcntl
    import struct
    locked = struct.unpack('i', fcntl.ioctl(slave, TIOCGEXCL, b'\0' * 4))[0]
    if locked and not in_use_by_others(pts):
        fcntl.ioctl(slave, termios.TIOCNXCL)
        print(f'Cleared a stale exclusive lock on {LINK}', flush=True)


def main():
    host, port = sys.argv[1], int(sys.argv[2])
    try:
        sock = socket.create_connection((host, port), timeout=3)
    except OSError as e:
        sys.exit(f'Could not connect to {host}:{port}: {e}')
    sock.settimeout(None)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    master, slave = os.openpty()
    tty.setraw(slave)
    pts = os.ttyname(slave)
    os.chmod(pts, 0o666)
    if os.path.lexists(LINK):
        os.remove(LINK)
    os.symlink(pts, LINK)

    def cleanup(*_):
        if os.path.realpath(LINK) == pts:
            os.remove(LINK)
        sys.exit(0)
    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)

    sel = selectors.DefaultSelector()
    sel.register(master, selectors.EVENT_READ, 'pty')
    sel.register(sock, selectors.EVENT_READ, 'tcp')
    print(f'{LINK} -> {pts} <-> {host}:{port}', flush=True)
    try:
        while True:
            events = sel.select(timeout=0.5)
            for key, _ in events:
                if key.data == 'pty':
                    sock.sendall(os.read(master, 4096))
                else:
                    data = sock.recv(4096)
                    if not data:
                        print('The Mac bridge closed the connection', flush=True)
                        return 1
                    os.write(master, data)
            if not events:
                clear_stale_lock(slave, pts)
    except OSError as e:
        print(f'Bridge error: {e}', flush=True)
        return 1
    finally:
        if os.path.realpath(LINK) == pts:
            os.remove(LINK)


if __name__ == '__main__':
    sys.exit(main())
