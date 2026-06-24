# Lite3 RL Deploy Zig

语言：[English](README.md) | 中文

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
| output | `policy_action` | `[12]` | 神经网络原始 action |

当前 Zig 版本直接对齐原 C++ 部署行为：ONNX Runtime 读取 `policy_action`，Zig 侧 clip 到 `[-12, 12]`，乘以 `0.25` action scale，加 policy 默认关节 `{0, -1, 1.8}`，最后做 joint target clamp。

观测向量 117 维布局：`commands(3) | rpy(3) | base_angular_velocity(3) | qpos(12) | qvel(12) | position_history(3×12) | velocity_history(2×12) | action_history_offset(2×12)`。DOF position/history 字段是绝对关节角，和原 C++ ONNX runner 一致。

关节顺序：`FL, FR, HL, HR` × `HipX, HipY, Knee`。

默认策略路径：`policy/ppo/policy.onnx`。

默认值位置：

- 默认 policy 路径在 `src/types.zig` 的 `ControllerConfig` 中设置。
- 默认机器人 IP/端口也在 `ControllerConfig` 中设置：`192.168.2.1:43893`。

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

## 机器人网络与 command console 设置

MotionSDK 使用双向 UDP：

- `lite3-deploy` 在本机 UDP `43897` 端口接收机器人状态。
- `lite3-deploy` 向机器人运动主机 / command console 的 `--robot-ip --robot-port` 下发关节指令，默认是 `192.168.2.1:43893`。

如果从开发 PC 运行，请先在机器人运动主机上配置：

```bash
ssh ysc@192.168.2.1   # 或替换为实际 Lite3 用户/主机
cd ~/jy_exe/conf
vim network.toml
```

典型 `network.toml`：

```toml
ip = '192.168.2.xxx'  # 运行 lite3-deploy 的机器 IP
target_port = 43897   # 该机器接收机器人状态的端口
local_port = 43893    # 机器人侧 SDK command 接收端口
```

如果 `lite3-deploy` 跑在开发 PC 上，`ip` 填开发 PC 的静态 IP；如果跑在机器人运动主机上，`ip` 填运动主机 IP，Lite3 WiFi 下通常是 `192.168.2.1`，`192.168.1.*`/有线网段通常是 `192.168.1.120`。`local_port` 需要和启动参数 `--robot-port` 一致。

修改后重启机器人运动程序：

```bash
cd ~/jy_exe/scripts
sudo ./stop.sh
sudo ./restart.sh
```

如果从 PC 运行，还要确认 PC 防火墙允许 UDP `43897` 入站，然后用运动主机 command endpoint 启动：

```bash
zig-out/bin/lite3-deploy \
  --policy policy/ppo/policy.onnx \
  --robot-ip <motion-host-ip> \
  --robot-port 43893
```

## 运行

> 真机运行前请确认机器人悬空/安全支撑、急停可用，并连接 Lite3 WiFi。

```bash
zig build run -- \
  --policy policy/ppo/policy.onnx \
  --robot-ip 192.168.2.1
```

默认命令输入是原版 Retroid UDP 手柄，端口 `12121`（`Y` 站立、`A` 进入 RL、左右摇杆同时按下阻尼）。stdin 键盘控制可加 `--keyboard`；如果 stdin 是 TTY，会启用 raw mode，不需要按 Enter：

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
  --policy policy/ppo/policy.onnx \
  --auto-rl \
  --fixed-command 0.2 0.0 0.0

# 指定 PD 增益与 policy decimation
zig-out/bin/lite3-deploy --policy policy/ppo/policy.onnx --kp 17 --kd 0.9 --decimation 12
```

## 部署到机器人

```bash
ROBOT_HOST=ysc@192.168.2.1 ./scripts/deploy_to_robot.sh
```

机器人端运行：

```bash
cd ~/Lite3_rl_deploy_zig
LD_LIBRARY_PATH=lib ./bin/lite3-deploy --policy policy/ppo/policy.onnx
```

## Zig 库用法

在其他 Zig 项目中可通过 `build.zig.zon` dependency 引入本库，然后：

```zig
const lite3 = @import("lite3_deploy");

var controller = try lite3.Controller.init(allocator, .{
    .policy_path = "policy/ppo/policy.onnx",
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
