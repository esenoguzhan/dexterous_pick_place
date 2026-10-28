#!/bin/bash
# demo.sh [restore]: the whole Restore demo on aras PC, with the checks that bit us on 2026-10-28.
#
#   demo.sh          check everything, record the order, wait for the camera to be covered and
#                    uncovered, put every object back, check the result
#   demo.sh restore  check everything, then restore to the order recorded last time (no waiting)
#
# Run it on aras PC (not in the container). Stop it with Ctrl+C; the arm then stays where it is.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
JETSON=http://169.254.211.202:8090
export WENDY_DEVICE=169.254.211.202:50051

say() { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
fail() { printf '\n\033[1;31mSTOP: %s\033[0m\n' "$*"; exit 1; }
robot_state() {
  docker exec restore_main bash -c "LD_LIBRARY_PATH=/ws/build/neura_driver/neura_bundle/usr/lib timeout 20 /run_data/robot_state $*" 2>&1 \
    | grep -v -E "Factory|RobotClient|^\s*$"
}

say "1/6 container and networks"
docker start restore_main >/dev/null || fail "the restore_main container does not exist (see docs/restore_physical_handoff.md)"
ping -c 1 -W 2 192.168.2.13 >/dev/null || fail "arm 192.168.2.13 not reachable: robot control box on? Ethernet to the robot switch plugged in?"
ping -c 1 -W 2 192.168.2.75 >/dev/null || fail "hand 192.168.2.75 not reachable: hand powered?"
echo "arm and hand reachable"

say "2/6 cameras (Jetson app perception-test)"
cams() { curl -s -m 6 $JETSON/api/status | python3 -c 'import json,sys; d=json.load(sys.stdin); print(" ".join(f"{c[\"id\"]}:{c[\"fps\"]:.0f}fps:usb{c[\"usb\"]}" for c in d["cameras"]))' 2>/dev/null; }
c=$(cams)
if ! echo "$c" | grep -q "263522073705:[1-9]"; then
  echo "camera app not answering; (re)starting it"
  timeout 30 wendy device apps stop restore-perception >/dev/null 2>&1
  timeout 30 wendy device apps stop perception-test >/dev/null 2>&1; sleep 3
  timeout 30 wendy device apps start perception-test --detach >/dev/null 2>&1
  for i in $(seq 1 25); do c=$(cams); echo "$c" | grep -q "263522073705:[1-9]" && break; sleep 3; done
fi
echo "$c" | grep -q "263522073705:[1-9]" || fail "no fixed camera: power-cycle the Jetson (USB-C to aras PC), wait ~1 min, run again"
echo "$c"
echo "$c" | grep -q "262722075125:[1-9]" || echo "note: no wrist camera; the grasp then uses the fixed camera only"

say "3/6 arm state"
s=$(robot_state | grep "^status")
echo "$s"
if echo "$s" | grep -q -E "collision/b6=1|error/b5=1"; then
  echo "collision or error flagged: check nothing is in contact, then clearing it"
  "$HERE/stack.sh" stop >/dev/null
  robot_state reset | grep -E "now"
  s=$(robot_state | grep "^status")
fi
echo "$s" | grep -q "power/b0=1" || fail "arm motors off: power it on at the pendant"
echo "$s" | grep -q "external/b4=1" || fail "arm not in external mode: on the pendant switch to External (NEURA C++) mode, Automatic"

say "4/6 controllers"
"$HERE/stack.sh" restart | tail -1

say "5/6 arm follows at full speed?"
if ! "$HERE/rrun.sh" jog.py 6 0.15 2.5 2>&1 | grep -E "there|back"; then
  robot_state | grep scaling
  fail "the arm does not keep up (speed capped, e.g. after a collision): on the pendant switch to manual mode, then back to Automatic (External on), and run again"
fi

say "6/6 Restore"
cd "$HERE"
if [ "${1:-}" = "restore" ]; then
  ./rrun.sh restore.py run
else
  echo "watch for: COVER THE CAMERA NOW -> cover the fixed camera, shuffle, uncover, step back"
  ./rrun.sh restore.py demo
fi
echo
echo "log: ~/restore_run/restore_log.txt   images: ~/restore_run/scenes/"
