#!/usr/bin/env bash
set -euo pipefail

ROBOT_HOST=${ROBOT_HOST:-ysc@192.168.2.1}
REMOTE_DIR=${REMOTE_DIR:-~/Lite3_rl_deploy_zig}
PLATFORM=${PLATFORM:-aarch64}
OPTIMIZE=${OPTIMIZE:-ReleaseFast}

zig build -Dplatform="${PLATFORM}" -Doptimize="${OPTIMIZE}"

ssh "${ROBOT_HOST}" "mkdir -p ${REMOTE_DIR}"
scp -r zig-out/bin zig-out/lib policy "${ROBOT_HOST}:${REMOTE_DIR}/"

echo "Deployed to ${ROBOT_HOST}:${REMOTE_DIR}"
echo "Run on robot: cd ${REMOTE_DIR} && LD_LIBRARY_PATH=lib ./bin/lite3-deploy --policy policy/deploy/lite3_policy.onnx"
