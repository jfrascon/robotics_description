#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_CLOSED_STL="${SCRIPT_DIR}/fork_simple_closed_tines.stl"
OUTPUT_OPEN_STL="${SCRIPT_DIR}/fork_simple_open_tines.stl"
TINE_LEN_X="1.20"
TINE_LEN_Y="0.12"
TINE_LEN_Z="0.06"
TINE_SEPARATION="0.25"
TINE_UNION_LEN_X="0.03"

python3 "${SCRIPT_DIR}/fork_simple_stl_generator.py" \
    --tine-len-x "${TINE_LEN_X}" \
    --tine-len-y "${TINE_LEN_Y}" \
    --tine-len-z "${TINE_LEN_Z}" \
    --tine-separation "${TINE_SEPARATION}" \
    --tine-union-len-x "${TINE_UNION_LEN_X}" \
    --output "${OUTPUT_CLOSED_STL}"

python3 "${SCRIPT_DIR}/fork_simple_stl_generator.py" \
    --tine-len-x "${TINE_LEN_X}" \
    --tine-len-y "${TINE_LEN_Y}" \
    --tine-len-z "${TINE_LEN_Z}" \
    --tine-separation "${TINE_SEPARATION}" \
    --tine-union-len-x "${TINE_UNION_LEN_X}" \
    --open-tines \
    --output "${OUTPUT_OPEN_STL}"
