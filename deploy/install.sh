#!/bin/bash
# Installs the F9P-Relay udev rule and systemd service. Run with sudo.
set -e

SRC="/home/karidrone/F9P-Relay/deploy"
RULES="/etc/udev/rules.d/99-f9p-relay.rules"
SERVICE="/etc/systemd/system/f9p-relay.service"

echo "[1/5] Installing udev rules -> $RULES"
cp "$SRC/99-f9p-relay.rules" "$RULES"
udevadm control --reload
udevadm trigger --subsystem-match=tty

echo "[2/5] Adding karidrone to dialout group"
usermod -aG dialout karidrone

echo "[3/5] Installing systemd service -> $SERVICE"
cp "$SRC/f9p-relay.service" "$SERVICE"
systemctl daemon-reload

echo "[4/5] Enabling service on boot"
systemctl enable f9p-relay.service

echo "[5/5] Starting service now"
systemctl restart f9p-relay.service

echo "Done."
