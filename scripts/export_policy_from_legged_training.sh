#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <checkpoint.eqx> [output.onnx] [metadata.json] [extra export args...]" >&2
  exit 2
fi

CHECKPOINT=$1
shift
OUTPUT=${1:-policy/deploy/lite3_policy.onnx}
if [[ $# -gt 0 ]]; then shift; fi
METADATA=${1:-policy/deploy/lite3_policy.metadata.json}
if [[ $# -gt 0 ]]; then shift; fi

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TRAINING_ROOT=$(cd -- "${REPO_ROOT}/../legged-training" && pwd)

if [[ "${OUTPUT}" = /* ]]; then
  OUTPUT_ABS=${OUTPUT}
else
  OUTPUT_ABS=${REPO_ROOT}/${OUTPUT}
fi
if [[ "${METADATA}" = /* ]]; then
  METADATA_ABS=${METADATA}
else
  METADATA_ABS=${REPO_ROOT}/${METADATA}
fi

mkdir -p "$(dirname "${OUTPUT_ABS}")" "$(dirname "${METADATA_ABS}")"

cd "${TRAINING_ROOT}"
uv run python scripts/export_policy_onnx.py \
  --policy-path "${CHECKPOINT}" \
  --output-path "${OUTPUT_ABS}" \
  --metadata-path "${METADATA_ABS}" \
  "$@"
