#!/bin/bash
# Build the Safe Semantic Planner as a WASM module using Emscripten.
# Requires emcc (emsdk) on PATH.
#
# Output: wasm/planner.js + wasm/planner.wasm
#
# Usage:
#   bash build_wasm.sh          # Standard build
#   bash build_wasm.sh --debug  # Debug build with assertions

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

OUT_DIR="wasm"
mkdir -p "$OUT_DIR"

OPTIMIZATION="-O2"
if [[ "${1:-}" == "--debug" ]]; then
    OPTIMIZATION="-O0 -g -s ASSERTIONS=2"
fi

SOURCES=(
    src/problem_loader.cpp
    src/kd_tree.cpp
    src/dstar_lite.cpp
    src/wasm_api.cpp
)

EXPORTED_FUNCTIONS="[
    '_create_planner',
    '_notify_edge_changed',
    '_notify_goal_changed',
    '_notify_bad_states_changed',
    '_recompute_with_weights',
    '_get_metrics',
    '_get_result',
    '_destroy_planner',
    '_malloc',
    '_free'
]"

EXPORTED_RUNTIME="[
    'ccall',
    'cwrap',
    'UTF8ToString',
    'stringToUTF8',
    'lengthBytesUTF8'
]"

echo "=== Building WASM module ==="
emcc \
    -std=c++14 \
    $OPTIMIZATION \
    -I include \
    "${SOURCES[@]}" \
    -s WASM=1 \
    -s MODULARIZE=1 \
    -s EXPORT_NAME="'PlannerModule'" \
    -s EXPORTED_FUNCTIONS="$EXPORTED_FUNCTIONS" \
    -s EXPORTED_RUNTIME_METHODS="$EXPORTED_RUNTIME" \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s INITIAL_MEMORY=16777216 \
    -s NO_EXIT_RUNTIME=1 \
    -s ENVIRONMENT='web,node' \
    --no-entry \
    -o "$OUT_DIR/planner.js"

echo "=== Build complete ==="
echo "Output: $OUT_DIR/planner.js + $OUT_DIR/planner.wasm"
ls -lh "$OUT_DIR"/planner.*
