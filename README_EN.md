# Lite3 RL Deploy Zig

Zig 0.16.0 Lite3 RL deployment library and executable. The old CMake/C++ and MuJoCo deployment pipeline has been removed.

Runtime dependencies:

- Lite3 Motion SDK: fetched as a Zig package dependency (see `build.zig.zon`)
- `third_party/onnxruntime/` for ONNX Runtime C shared libraries

## Expected ONNX

The deploy ONNX must match this I/O contract:

| item | name | shape | notes |
|---|---|---:|---|
| input 0 | `raw_obs` | `[B, 117]` | unnormalized observation |
| input 1 | `raw_obs_history` | `[B, 40, 117]` | unnormalized observation history |
| output | `joint_target` | `[B, 12]` | 12 joint PD targets in radians |

117-dim observation layout: `commands(3) | rpy(3) | base_angular_velocity(3) | qpos(12) | qvel(12) | position_history(3x12) | velocity_history(2x12) | action_history(2x12)`.

Joint order: `FL, FR, HL, HR` x `HipX, HipY, Knee`.

Default policy path: `policy/deploy/lite3_policy.onnx`.

## Build

```bash
zig version   # must be 0.16.0
zig build
zig build -Doptimize=ReleaseFast
zig build -Dplatform=aarch64 -Doptimize=ReleaseFast
```

`zig-out/lib` includes ONNX Runtime and Lite3 Motion SDK shared libraries.

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