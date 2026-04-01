#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_STL="${SCRIPT_DIR}/rectangular_body.stl"
BODY_LEN_X="1.044"
BODY_LEN_Y="0.650"
BODY_LEN_Z="0.235"
WALL_THICKNESS="0.001"

python3 "${SCRIPT_DIR}/rectangular_body_stl_generator.py" \
    --body-len-x "${BODY_LEN_X}" \
    --body-len-y "${BODY_LEN_Y}" \
    --body-len-z "${BODY_LEN_Z}" \
    --wall-thickness "${WALL_THICKNESS}" \
    --output "${OUTPUT_STL}"
