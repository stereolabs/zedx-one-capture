#!/bin/bash
# Build the library + test app and run. Usage:
#   ./build_and_run.sh [camera_id] [settle_frames]
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB_DIR="$SCRIPT_DIR/../lib"

echo "=== Building library ==="
cd "$LIB_DIR"
mkdir -p build && cd build
cmake -DCMAKE_CXX_FLAGS="-L/usr/lib/aarch64-linux-gnu/nvidia" .. 2>&1 | tail -3
make -j$(nproc)

echo "=== Building test app ==="
cd "$SCRIPT_DIR"
mkdir -p build && cd build
cmake .. 2>&1 | tail -3
make -j$(nproc)

echo "=== Running WB test ==="
export LD_LIBRARY_PATH="$LIB_DIR/build:$LD_LIBRARY_PATH"
./wb_test "$@"
