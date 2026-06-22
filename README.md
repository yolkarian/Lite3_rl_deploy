# Lite3 RL Deploy Zig

这是一个 **Zig 0.16.0** 版 Lite3 强化学习策略部署库/可执行程序。项目已移除 CMake/C++ 与 MuJoCo 部署链路，运行时只保留：

- `../Lite3_MotionSDK_Zig/`：Lite3 Zig/C wrapper motion SDK（构建时作为 Zig dependency 引入）
- `third_party/onnxruntime/`：ONNX Runtime C 动态库
- `src/`：Zig 部署库与 `lite3-deploy` 可执行程序

## ONNX 格式

默认适配 `../legged-training/scripts/export_policy_onnx.py` 导出的 deploy graph：

| 项 | 名称 | shape | 说明 |
|---|---|---:|---|
| input 0 | `raw_obs` | `[B, 117]` | 未归一化观测 |
| input 1 | `raw_obs_history` | `[B, 40, 117]` | 未归一化观测历史 |
| output | `joint_target` | `[B, 12]` | 12 个关节 PD target，单位 rad |

关节顺序与 `legged-training/configs/env/lite3.yaml` 一致：`FL, FR, HL, HR` × `HipX, HipY, Knee`。

## 构建

```bash
zig version   # 必须是 0.16.0
zig build     # x86_64 本机调试构建
zig build -Doptimize=ReleaseFast

# 交叉构建机器人端 aarch64 Linux
zig build -Dplatform=aarch64 -Doptimize=ReleaseFast
```

构建产物：

- `zig-out/lib/liblite3_deploy.a`
- `zig-out/bin/lite3-deploy`
- `zig-out/lib/*.so`：已复制 ONNX Runtime 与 Lite3 Motion SDK 动态库

## 从 legged-training 导出策略

```bash
./scripts/export_policy_from_legged_training.sh \
  ../legged-training/outputs/<run>/checkpoints/latest.eqx
```

默认输出：

```text
policy/deploy/lite3_policy.onnx
policy/deploy/lite3_policy.metadata.json
```

## 运行

> 真机运行前请确认机器人悬空/安全支撑、急停可用，并连接 Lite3 WiFi。

```bash
zig build run -- \
  --policy policy/deploy/lite3_policy.onnx \
  --robot-ip 192.168.2.1
```

键盘控制需要按 Enter 生效：

- `z`：Idle → StandUp
- `c`：StandUp → RLControl
- `r`：进入 JointDamping
- `w/s`：前后速度
- `a/d`：侧向速度
- `q/e`：yaw 速度
- `x`：速度清零

常用参数：

```bash
# 自动站立并进入 RL，固定速度命令
zig-out/bin/lite3-deploy \
  --policy policy/deploy/lite3_policy.onnx \
  --auto-rl \
  --fixed-command 0.2 0.0 0.0

# 指定 PD 增益与 policy decimation
zig-out/bin/lite3-deploy --policy policy/deploy/lite3_policy.onnx --kp 20 --kd 0.7 --decimation 12
```

## 部署到机器人

```bash
ROBOT_HOST=ysc@192.168.2.1 ./scripts/deploy_to_robot.sh
```

机器人端运行：

```bash
cd ~/Lite3_rl_deploy_zig
LD_LIBRARY_PATH=lib ./bin/lite3-deploy --policy policy/deploy/lite3_policy.onnx
```

## Zig 库用法

在其他 Zig 项目中可通过 `build.zig.zon` path dependency 引入本库，然后：

```zig
const lite3 = @import("lite3_deploy");

var controller = try lite3.Controller.init(allocator, .{
    .policy_path = "policy/deploy/lite3_policy.onnx",
    .robot_ip = "192.168.2.1",
});
defer controller.deinit();
try controller.run();
```

## 已移除内容

- MuJoCo sim/deploy pipeline
- 旧 CMake/C++ state machine 与 policy runner
- 旧 `third_party/Lite3_MotionSDK`，改用 `../Lite3_MotionSDK_Zig/`
- PyTorch `pt2onnx.py`，改用 `../legged-training/scripts/export_policy_onnx.py`
