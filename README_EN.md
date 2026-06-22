# Lite3 RL Deploy Zig

Zig 0.16.0 Lite3 RL deployment library and executable. The old CMake/C++ and MuJoCo deployment pipeline has been removed.

Runtime dependencies:

- `../Lite3_MotionSDK_Zig/` as the Zig motion SDK dependency
- `third_party/onnxruntime/` for ONNX Runtime C shared libraries

## Expected ONNX

The runner targets ONNX files exported by `../legged-training/scripts/export_policy_onnx.py` with default deploy preprocessing/postprocessing:

- inputs: `raw_obs` `[B,117]`, `raw_obs_history` `[B,40,117]`
- output: `joint_target` `[B,12]` in radians

Joint order: `FL, FR, HL, HR` × `HipX, HipY, Knee`.

## Build

```bash
zig version   # must be 0.16.0
zig build
zig build -Doptimize=ReleaseFast
zig build -Dplatform=aarch64 -Doptimize=ReleaseFast
```

`zig-out/lib` includes ONNX Runtime and Lite3 Motion SDK shared libraries.

## Export a policy

```bash
./scripts/export_policy_from_legged_training.sh \
  ../legged-training/outputs/<run>/checkpoints/latest.eqx
```

Default output: `policy/deploy/lite3_policy.onnx`.

## Run

```bash
zig build run -- --policy policy/deploy/lite3_policy.onnx --robot-ip 192.168.2.1
```

Keyboard commands require Enter: `z` stand, `c` RL, `r` damping, `wasd` xy velocity, `q/e` yaw, `x` zero velocity.

Deploy to robot:

```bash
ROBOT_HOST=ysc@192.168.2.1 ./scripts/deploy_to_robot.sh
```

Robot side:

```bash
cd ~/Lite3_rl_deploy_zig
LD_LIBRARY_PATH=lib ./bin/lite3-deploy --policy policy/deploy/lite3_policy.onnx
```
