# AGENTS.md

This repository is a Zig 0.16.0 Lite3 RL deployment library/executable.

## Build

```bash
zig build
zig build -Doptimize=ReleaseFast
zig build -Dplatform=aarch64 -Doptimize=ReleaseFast
zig build test
```

## Architecture

- `build.zig`: Zig package build; imports `../Lite3_MotionSDK_Zig` as dependency and installs required dynamic libraries.
- `src/motion.zig`: hardware wrapper over `lite3_motion_sdk` translated C API.
- `src/onnx_policy.zig`: ONNX Runtime C API wrapper.
- `src/observation.zig`: legged-training deploy observation layout (`raw_obs` 117, history 40x117).
- `src/controller.zig`: Idle → StandUp → RLControl → JointDamping state machine.
- `src/main.zig`: `lite3-deploy` CLI.

## Policy format

Use ONNX exported by `../legged-training/scripts/export_policy_onnx.py`:

- inputs: `raw_obs`, `raw_obs_history`
- output: `joint_target`

Default path: `policy/deploy/lite3_policy.onnx`.

## Removed legacy pieces

Do not reintroduce the old CMake/C++ or MuJoCo deploy pipeline. The project should stay Zig-first and use `../Lite3_MotionSDK_Zig` rather than the old vendored Lite3 Motion SDK.
