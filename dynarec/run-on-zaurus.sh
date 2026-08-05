#!/bin/sh
# run-on-zaurus.sh -- build a dynarec-gated PocketSNES, copy it to a real
# Zaurus, run it there, and bring the output back.
#
# Why this exists: the dynarec emits ARMv5, so it can only be exercised on ARM.
# Two ways to do that, and only two:
#
#   full-system QEMU   tools/build-and-emulate.sh in piko, or debugfs-inject
#                      into piko-emulator-sd.img and boot -M husky. Works, slow.
#   the device         this script.
#
# User-mode qemu-arm is NOT a third option. PocketSNES segfaults under it with
# the dynarec disarmed, so it is not a dynarec problem -- see commit 15e6c60,
# which documented a second incompatibility beyond the framebuffer.
#
# The device needs dropbear up (password root/zaurus, or a key) and WiFi
# associated. It does NOT need piko-sync-server: that is only required by
# piko's tools/build-and-deploy.sh, which speaks its own TCP protocol. Plain
# scp is enough for one binary.
#
# Usage:
#   dynarec/run-on-zaurus.sh [-m MODE] [-w SECONDS] [-r ROM] root@HOST
#
#   -m  exec | verify | profile | none      (default: exec)
#   -w  wall-clock seconds to run           (default: 120)
#   -r  ROM path on the BUILD HOST          (default: the FF6 image used by
#                                            arm-snesrec, copied if absent)
#
# Example:
#   dynarec/run-on-zaurus.sh -m verify -w 180 root@10.208.47.72
set -eu

MODE=exec
WALL=120
ROM="$HOME/Code/arm-snesrec/roms/ff6.smc"
PIKO="${PIKO_DIR:-$HOME/Code/piko}"

while getopts "m:w:r:" o; do
	case "$o" in
		m) MODE=$OPTARG ;;
		w) WALL=$OPTARG ;;
		r) ROM=$OPTARG ;;
		*) echo "see the header for usage" >&2; exit 2 ;;
	esac
done
shift $((OPTIND - 1))
[ $# -eq 1 ] || { echo "usage: $0 [-m MODE] [-w SECS] [-r ROM] root@HOST" >&2; exit 2; }
TARGET=$1
HOST=${TARGET#*@}

case "$MODE" in
	exec)    BUILDFLAG=EXEC=1;    GATE=PIKO_DYN_EXEC=1    ;;
	verify)  BUILDFLAG=VERIFY=1;  GATE=PIKO_DYN_VERIFY=1  ;;
	profile) BUILDFLAG=PROFILE=1; GATE=PIKO_DYN_PROFILE=1 ;;
	none)    BUILDFLAG=;          GATE=                   ;;
	*) echo "unknown MODE '$MODE'" >&2; exit 2 ;;
esac

# Reachability first. A deploy that dies halfway leaves a truncated binary on
# the device, which then fails in a way that looks like a dynarec bug.
echo "== checking $HOST"
ping -c1 -W2 "$HOST" >/dev/null 2>&1 || {
	echo "$HOST does not answer. Power the board on, and check WiFi has" >&2
	echo "associated -- a static 10.208.47.22 instead of a DHCP lease is" >&2
	echo "what a broken data path looks like (piko AGENTS.md)." >&2
	exit 1
}

export PATH="$PIKO/toolchain/x-tools/arm-unknown-linux-uclibcgnueabi/bin:$PATH"
echo "== building (${BUILDFLAG:-no dynarec gate})"
make -f Makefile.zaurus clean >/dev/null
# shellcheck disable=SC2086
make -f Makefile.zaurus $BUILDFLAG PIKO_DIR="$PIKO" >/dev/null
echo "   $(ls -l PocketSNES | awk '{print $5}') bytes"

echo "== copying to $TARGET:/root"
scp -O PocketSNES "$TARGET:/root/PocketSNES-dyn"
ssh "$TARGET" 'chmod +x /root/PocketSNES-dyn'
# The ROM is large and rarely changes, so only send it when it is missing.
if ! ssh "$TARGET" 'test -f /root/ff6.smc'; then
	echo "== copying ROM (once)"
	scp -O "$ROM" "$TARGET:/root/ff6.smc"
fi

echo "== running MODE=$MODE for ${WALL}s"
# SAL_HEADLESS is deliberately NOT set: on the device there IS a framebuffer,
# and running through it is the point -- this is the first configuration where
# the w100 blit and the dynarec are exercised together.
ssh "$TARGET" "cd /root && PIKO_TRACE_MASH=1 ${GATE} \
	timeout ${WALL} ./PocketSNES-dyn /root/ff6.smc 2>&1; \
	echo \"=== exited rc=\$? ===\"" | tee "zaurus-${MODE}.log"

echo
echo "== summary"
grep -E "DYN-(EXEC|VERIFY|PROFILE)|diverged|exited rc" "zaurus-${MODE}.log" | tail -12
echo "   full output in zaurus-${MODE}.log"
