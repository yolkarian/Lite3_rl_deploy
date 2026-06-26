const std = @import("std");
const deploy = @import("lite3_deploy");

const OptionsError = error{ MissingValue, InvalidValue };

pub fn main(init: std.process.Init) !void {
    var it = try std.process.Args.Iterator.initAllocator(init.minimal.args, init.gpa);
    defer it.deinit();

    const config = try parseArgs(&it);
    var controller = try deploy.Controller.init(init.gpa, config);
    defer controller.deinit();
    try controller.run();
}

fn parseArgs(it: *std.process.Args.Iterator) !deploy.types.ControllerConfig {
    var config = deploy.types.ControllerConfig{
        .command_mode = .keyboard,
        .keyboard_gain_tuning = true,
        .fixed_command = .{ 0.0, 0.0, 0.0 },
    };

    _ = it.skip();
    while (it.next()) |arg_z| {
        const arg: []const u8 = arg_z;
        if (std.mem.eql(u8, arg, "--help") or std.mem.eql(u8, arg, "-h")) {
            printHelp();
            std.process.exit(0);
        } else if (std.mem.eql(u8, arg, "--policy")) {
            config.policy_path = try nextValue(it);
        } else if (std.mem.eql(u8, arg, "--robot-ip")) {
            config.robot_ip = try nextValue(it);
        } else if (std.mem.eql(u8, arg, "--robot-port")) {
            config.robot_port = try std.fmt.parseInt(u16, try nextValue(it), 10);
        } else if (std.mem.eql(u8, arg, "--decimation")) {
            config.policy_decimation = try std.fmt.parseInt(u32, try nextValue(it), 10);
        } else if (std.mem.eql(u8, arg, "--auto-rl")) {
            config.auto_rl = true;
        } else if (std.mem.eql(u8, arg, "--max-seconds")) {
            config.max_run_time_s = try std.fmt.parseFloat(f64, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--clip-actions")) {
            config.clip_actions = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kp")) {
            config.rl_gains.kp = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kd")) {
            config.rl_gains.kd = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kp-step")) {
            config.gain_kp_step = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kd-step")) {
            config.gain_kd_step = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kp-range")) {
            config.gain_kp_min = try std.fmt.parseFloat(f32, try nextValue(it));
            config.gain_kp_max = try std.fmt.parseFloat(f32, try nextValue(it));
            if (config.gain_kp_min > config.gain_kp_max) return OptionsError.InvalidValue;
        } else if (std.mem.eql(u8, arg, "--kd-range")) {
            config.gain_kd_min = try std.fmt.parseFloat(f32, try nextValue(it));
            config.gain_kd_max = try std.fmt.parseFloat(f32, try nextValue(it));
            if (config.gain_kd_min > config.gain_kd_max) return OptionsError.InvalidValue;
        } else {
            std.debug.print("unknown argument: {s}\n", .{arg});
            printHelp();
            return OptionsError.InvalidValue;
        }
    }

    config.command_mode = .keyboard;
    config.keyboard_gain_tuning = true;
    config.fixed_command = .{ 0.0, 0.0, 0.0 };
    config.rl_gains.kp = deploy.types.clamp(config.rl_gains.kp, config.gain_kp_min, config.gain_kp_max);
    config.rl_gains.kd = deploy.types.clamp(config.rl_gains.kd, config.gain_kd_min, config.gain_kd_max);
    return config;
}

fn nextValue(it: *std.process.Args.Iterator) ![]const u8 {
    return it.next() orelse OptionsError.MissingValue;
}

fn printHelp() void {
    std.debug.print(
        \\Lite3 Kp/Kd gain tuning deploy
        \\
        \\Usage:
        \\  zig build run-gain-tune -- [options]
        \\  zig-out/bin/lite3-gain-tune [options]
        \\
        \\Behavior:
        \\  Same Idle -> StandUp -> RLControl flow as lite3-deploy.
        \\  In RLControl the velocity command is fixed to (0, 0, 0).
        \\  Keyboard runs in raw nonblocking mode when possible, so no Enter is needed.
        \\
        \\Options:
        \\  --policy PATH                 default: policy/ppo/policy.onnx
        \\  --robot-ip IP                 default: 192.168.1.120
        \\  --robot-port PORT             default: 43893
        \\  --decimation N                default: 12
        \\  --kp VALUE --kd VALUE          initial RL PD gains (default 17 / 0.9)
        \\  --kp-step VALUE                keyboard Kp step (default 1.0)
        \\  --kd-step VALUE                keyboard Kd step (default 0.1)
        \\  --kp-range MIN MAX             clamp Kp (default 0 / 200)
        \\  --kd-range MIN MAX             clamp Kd (default 0 / 20)
        \\  --clip-actions VALUE           default: 12.0
        \\  --auto-rl                     auto request StandUp then RLControl
        \\  --max-seconds SEC             exit after SEC seconds
        \\
        \\Keyboard:
        \\  z = stand, c = enter RL, r = damping
        \\  in RL: u/j = Kp +/- step, i/k = Kd +/- step, g = print gains
        \\
    , .{});
}
