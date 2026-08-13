#!/bin/sh
# install-on-zaurus.sh -- put the two desktop launchers on the device.
#
#   SSH="sshpass -p zaurus ssh" SCP="sshpass -p zaurus scp" \
#     zaurus/install-on-zaurus.sh root@10.208.47.1
#
# The launchers run /root/PocketSNES-dyn, which is where
# dynarec/run-on-zaurus.sh already deploys. Install a binary first, or the
# entries appear in the menu and do nothing.
set -eu
[ $# -eq 1 ] || { echo "usage: $0 root@HOST" >&2; exit 2; }
TARGET=$1
HOST=${TARGET#*@}
SSH="${SSH:-ssh}"
SCP="${SCP:-scp}"
HERE=$(dirname "$0")

ping -c1 -W2 "$HOST" >/dev/null 2>&1 || { echo "$HOST does not answer" >&2; exit 1; }

$SCP -O "$HERE/pocketsnes-run" "$TARGET:/usr/local/bin/pocketsnes-run"
$SSH "$TARGET" 'chmod +x /usr/local/bin/pocketsnes-run'
$SCP -O "$HERE/pocketsnes-interp.desktop"  "$TARGET:/usr/share/applications/"
$SCP -O "$HERE/pocketsnes-dynarec.desktop" "$TARGET:/usr/share/applications/"

# The rootfs is jffs2 mounted rw; sync so the entries survive a hard power-off,
# which on this device is a normal way to end a session.
$SSH "$TARGET" 'sync'
echo "installed. Both entries appear under Games in the panel menu."
echo "NOTE: the dynarec entry needs an EXEC=1 build at /root/PocketSNES-dyn --"
echo "      a default build ignores PIKO_DYN_EXEC and silently interprets."
