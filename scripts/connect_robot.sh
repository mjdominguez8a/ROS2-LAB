#!/usr/bin/env bash
# MacBook only: create /dev/so100 inside the container from the serial bridge running on the Mac
# (uv run --with pyserial scripts/mac_serial_bridge.py). See docs/REAL_ROBOT.md > MacBook.
#   bash scripts/connect_robot.sh          connect
#   bash scripts/connect_robot.sh --stop   disconnect
set -e
HOST="${SO100_BRIDGE_HOST:-host.docker.internal}"
PORT="${SO100_BRIDGE_PORT:-7001}"
PIDFILE=/tmp/so100_bridge.pid
LOG=/tmp/so100_bridge.log
BRIDGE="$(cd "$(dirname "$0")" && pwd)/pty_bridge.py"

stop() {
  if [ -f "$PIDFILE" ]; then
    sudo kill "$(cat "$PIDFILE")" 2>/dev/null || true
    sudo rm -f "$PIDFILE"
  fi
  sudo pkill -f '[s]ocat .*link=/dev/so100' 2>/dev/null || true  # bridge from older versions of this script
  sudo pkill -f '[p]ty_bridge.py' 2>/dev/null || true             # [p]: don't match this pkill
  sudo rm -f /dev/so100
}

if [ "$1" = --stop ]; then stop; echo "Disconnected."; exit 0; fi

stop
# Runs as root: it creates /dev/so100 and must be able to clear the driver's leftover port lock.
sudo sh -c "python3 -I '$BRIDGE' '$HOST' '$PORT' >'$LOG' 2>&1 </dev/null & echo \$! > '$PIDFILE'"

for _ in $(seq 30); do
  if [ -e /dev/so100 ]; then echo "Robot connected at /dev/so100 (via $HOST:$PORT)"; exit 0; fi
  sleep 0.1
done
echo "Could not connect to $HOST:$PORT. Start the bridge on the Mac first:"
echo "    uv run --with pyserial scripts/mac_serial_bridge.py"
cat "$LOG" 2>/dev/null
stop
exit 1
