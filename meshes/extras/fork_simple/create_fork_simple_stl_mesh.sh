#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FORK_SIMPLE_DIR="$(dirname "$SCRIPT_DIR")"
GENERATOR_PY="${FORK_SIMPLE_DIR}/fork_simple_stl_generator.py"
OUTPUT_STL="${SCRIPT_DIR}/fork_simple_0.stl"

python3 "${GENERATOR_PY}" \
    --tine-len-x 1.20 \
    --tine-len-y 0.12 \
    --tine-len-z 0.06 \
    --tine-separation 0.25 \
    --tine-union-len-x 0.03 \
    --output "${OUTPUT_STL}"

echo "Wrote ${OUTPUT_STL}"
