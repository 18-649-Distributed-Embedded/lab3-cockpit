#!/usr/bin/env bash
set -euo pipefail

BITRATE="${1:-500000}"

for dev in can0 can1; do
  sudo ip link set "$dev" down 2>/dev/null || true
  sudo ip link set "$dev" up type can bitrate "$BITRATE" restart-ms 100
  ip -details link show "$dev"
done