#!/bin/sh
set -eu

RULE_FILE="/etc/udev/rules.d/ydlidar.rules"

cat > "${RULE_FILE}" <<'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", MODE:="0777", SYMLINK+="ydlidar"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", MODE:="0777", SYMLINK+="ydlidar"
KERNEL=="ttyACM*", MODE:="0777", SYMLINK+="ydlidar"
EOF

udevadm control --reload-rules
udevadm trigger

echo "YDLidar udev rule installed. Replug the lidar, then check /dev/ydlidar."
