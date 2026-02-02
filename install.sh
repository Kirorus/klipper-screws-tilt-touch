#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"
PRINTER_CONFIG_DIR="${PRINTER_CONFIG_DIR:-$HOME/printer_data/config}"

SRC_PY="$ROOT_DIR/klippy/extras/screws_tilt_touch.py"
DST_PY="$KLIPPER_DIR/klippy/extras/screws_tilt_touch.py"

SRC_CFG="$ROOT_DIR/config/screws-tilt-adjust.cfg"
DST_CFG="$PRINTER_CONFIG_DIR/config_packages/macro/screws-tilt-adjust.cfg"

echo "Installing screws_tilt_touch..."
echo " - Klipper: $SRC_PY -> $DST_PY"
echo " - Config : $SRC_CFG -> $DST_CFG"

mkdir -p "$(dirname "$DST_PY")"
mkdir -p "$(dirname "$DST_CFG")"

cp -f "$SRC_PY" "$DST_PY"
cp -f "$SRC_CFG" "$DST_CFG"

echo "Done."
echo "Restart Klipper (RESTART or FIRMWARE_RESTART)."

