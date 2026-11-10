#!/usr/bin/env bash
# Compile the ST controller with matiec and link it into a shared library that the Python plant
# model loads with ctypes (mount/plc.py).
#   MATIEC_DIR  matiec checkout with iec2c built (default: ~/matiec)
#   PYTHON      python interpreter (default: python3)
set -euo pipefail
cd "$(dirname "$0")/.."
MATIEC_DIR="${MATIEC_DIR:-$HOME/matiec}"
PYTHON="${PYTHON:-python3}"
OUT=build/plc
rm -rf "$OUT"
mkdir -p "$OUT/iec"
"$MATIEC_DIR/iec2c" -I "$MATIEC_DIR/lib" -T "$OUT/iec" plc/st/mount_cell.st
"$PYTHON" plc/gen_shim.py plc/st/mount_cell.st "$OUT" MOUNT_CELL
gcc -O2 -fPIC -shared -w -I "$OUT/iec" -I "$MATIEC_DIR/lib/C" -o "$OUT/libplc.so" "$OUT/shim.c" -lm
echo "built $OUT/libplc.so"
