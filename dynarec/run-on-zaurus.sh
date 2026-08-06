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
# The emulator is launched through matchbox-fbrun, and that is NOT optional
# when X is up. Xfbdev maps /dev/fb0 and holds an EVIOCGRAB on the keyboard
# and touchscreen, so a framebuffer app started under it renders but never
# receives a keystroke -- it looks alive and is completely uncontrollable.
# matchbox-fbrun stops the graphical session, gives the program the console,
# and restores everything afterwards unconditionally. -y skips its on-screen
# confirmation dialog, which is what we want from a script.
#
# BOUNDING THE RUN. There is nothing on the device to do it with. This rootfs
# has no `timeout` applet (fbrun reports that as a bare and very misleading
# "cannot run timeout: File exists"), and no `kill` either -- not a binary, not
# an ash builtin; `kill -9 $pid` exits 127. A device-side sleep/kill watchdog
# therefore slept, failed to kill anything, and exited, leaving the emulator to
# run until something else stopped it.
#
# Closing the ssh so SIGHUP lands works SOMETIMES and is not dependable -- it
# ended one run and left the next one going. So the emulator bounds ITSELF:
# PIKO_MAX_SECONDS (menu/main.cpp) exits the process after N seconds. The
# host-side `timeout` below is only a backstop.
#
# ALWAYS BUILD CLEAN. Makefile.zaurus does not track the dynarec -D flags, so
# switching between EXEC=1 / VERIFY=1 / PROFILE=1 silently reuses objects
# compiled under the previous set. That produced a run reporting "armed"
# followed by "0 blocks run", and another where EXEC and PROFILE were both on
# -- a combination that measures nothing, because a translated block jumps
# past the profiler hook. Three hardware measurements had to be thrown away
# before this was spotted, hence the unconditional `clean` below.
#
# Set SSH/SCP to override the transport, e.g. for password auth:
#   SSH="sshpass -p zaurus ssh" SCP="sshpass -p zaurus scp" ...
#
# Usage:
#   dynarec/run-on-zaurus.sh [-m MODE] [-w SECONDS] [-r ROM] root@HOST
#
#   -m  exec | verify | profile | none      (default: exec)
#   -w  wall-clock seconds to run           (default: 120)
#   -r  ROM path on the BUILD HOST          (default: the FF6 image used by
#                                            arm-snesrec, copied if absent)
#   -F  extra flags for matchbox-fbrun, e.g. -F '--qvga --fast-pll'
#
# Example:
#   dynarec/run-on-zaurus.sh -m verify -w 180 root@10.208.47.72
set -eu

MODE=exec
WALL=120
ROM="$HOME/Code/arm-snesrec/roms/ff6.smc"
PIKO="${PIKO_DIR:-$HOME/Code/piko}"
FBFLAGS=
SSH="${SSH:-ssh}"
SCP="${SCP:-scp}"
# The root filesystem is a ~70M jffs2 with single-digit megabytes free, so the
# ROM goes on the SD card and only the binary lands in /root.
DEV_ROM=/mnt/card/ff6.smc
DEV_BIN=/root/PocketSNES-dyn

while getopts "m:w:r:F:" o; do
	case "$o" in
		m) MODE=$OPTARG ;;
		w) WALL=$OPTARG ;;
		r) ROM=$OPTARG ;;
		F) FBFLAGS=$OPTARG ;;
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

echo "== copying to $TARGET:$DEV_BIN"
$SCP -O PocketSNES "$TARGET:$DEV_BIN"
$SSH "$TARGET" "chmod +x $DEV_BIN"
# The ROM is large and rarely changes, so only send it when it is missing.
if ! $SSH "$TARGET" "test -f $DEV_ROM"; then
	echo "== copying ROM to $DEV_ROM (once, ~4M)"
	$SCP -O "$ROM" "$TARGET:$DEV_ROM"
fi

echo "== running MODE=$MODE for ${WALL}s (via matchbox-fbrun${FBFLAGS:+ $FBFLAGS})"
echo "   the device screen will drop out of X for the duration"

# SAL_HEADLESS is deliberately NOT set: on the device there IS a framebuffer,
# and running through it is the point -- this is the first configuration where
# the w100 blit and the dynarec are exercised together.
RUNNER=$(mktemp)
cat > "$RUNNER" <<RUNEOF
#!/bin/sh
# generated by dynarec/run-on-zaurus.sh -- do not edit on the device
cd /root
export PIKO_TRACE_MASH=1
export PIKO_MAX_SECONDS=$WALL
${GATE:+export $GATE}
exec $DEV_BIN $DEV_ROM
RUNEOF
$SCP -O "$RUNNER" "$TARGET:/root/run-dyn-device.sh"
rm -f "$RUNNER"
$SSH "$TARGET" 'chmod +x /root/run-dyn-device.sh'

# The grace period covers fbrun stopping X and putting it back; the emulator
# itself is what the WALL bound applies to.
timeout $((WALL + 45)) $SSH "$TARGET" \
	"/usr/sbin/matchbox-fbrun -y -n PocketSNES ${FBFLAGS} -- \
	/bin/sh /root/run-dyn-device.sh 2>&1" | tee "zaurus-${MODE}.log" || true

echo "== letting the device settle, then confirming X came back"
sleep 8
$SSH "$TARGET" 'ps | grep -c "[X]fbdev"' | while read n; do
	[ "$n" -ge 1 ] && echo "   X restored" || echo "   WARNING: X did NOT come back"
done

echo
echo "== summary"
grep -E "DYN-(EXEC|VERIFY|PROFILE)|diverged|exited rc" "zaurus-${MODE}.log" | tail -12
echo "   full output in zaurus-${MODE}.log"
