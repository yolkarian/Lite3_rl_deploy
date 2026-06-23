# Lite3 RL Deploy Zig

这是一个 **Zig 0.16.0** 版 Lite3 强化学习策略部署库/可执行程序。项目已移除 CMake/C++ 与 MuJoCo 部署链路，运行时只保留：

- Lite3 Motion SDK：作为 Zig package dependency 引入（见 `build.zig.zon`）
- `third_party/onnxruntime/`：ONNX Runtime C 动态库
- `src/`：Zig 部署库与 `lite3-deploy` 可执行程序

## ONNX 格式

部署 ONNX 需匹配训练导出脚本默认生成的 single-sample I/O 约定（不要使用 batched/fixed-batch 导出）：

| 项 | 名称 | shape | 说明 |
|---|---|---:|---|
| input 0 | `raw_obs` | `[117]` | 未归一化观测 |
| input 1 | `raw_obs_history` | `[40, 117]` | 未归一化观测历史 |
| output | `joint_target` | `[12]` | 12 个关节 PD target，单位 rad |

该默认导出已在 ONNX 内包含观测 scale/clip、action clip/scale、default joint position 与 joint target clamp；Zig 侧只传入 raw observation，不再做 normalizer 处理。

观测向量 117 维布局：`commands(3) | rpy(3) | base_angular_velocity(3) | qpos(12) | qvel(12) | position_history(3×12) | velocity_history(2×12) | action_history(2×12)`。

关节顺序：`FL, FR, HL, HR` × `HipX, HipY, Knee`。

默认策略路径：`policy/deploy/lite3_policy.onnx`。

默认值位置：

- 默认 policy 路径在 `src/main.zig` 的 `parseArgs()` 中设置。
- 默认机器人 IP/端口在 `src/types.zig` 的 `ControllerConfig` 中设置：`192.168.2.1:43893`。

如需长期修改默认值，改上述源码；临时覆盖用命令行参数 `--policy`、`--robot-ip`、`--robot-port`。

## 构建

```bash
zig version   # 必须是 0.16.0
zig build     # x86_64 本机调试构建
zig build test # 包含默认 ONNX smoke test
zig build -Doptimize=ReleaseFast

# 交叉构建机器人端 aarch64 Linux
zig build -Dplatform=aarch64 -Doptimize=ReleaseFast
```

构建产物：

- `zig-out/lib/liblite3_deploy.a`
- `zig-out/bin/lite3-deploy`
- `zig-out/lib/*.so`：已复制 ONNX Runtime 与 Lite3 Motion SDK 动态库

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

在其他 Zig 项目中可通过 `build.zig.zon` dependency 引入本库，然后：

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
- 旧 vendored Lite3 Motion SDK，改用 Zig package dependency
- 旧 PyTorch `pt2onnx.py` 转换脚本