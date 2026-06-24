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
    var config = deploy.types.ControllerConfig{};

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
        } else if (std.mem.eql(u8, arg, "--command")) {
            config.command_mode = deploy.types.CommandMode.parse(try nextValue(it)) orelse return OptionsError.InvalidValue;
        } else if (std.mem.eql(u8, arg, "--keyboard")) {
            config.command_mode = .keyboard;
        } else if (std.mem.eql(u8, arg, "--no-command")) {
            config.command_mode = .none;
        } else if (std.mem.eql(u8, arg, "--gamepad-port")) {
            config.gamepad_port = try std.fmt.parseInt(u16, try nextValue(it), 10);
        } else if (std.mem.eql(u8, arg, "--decimation")) {
            config.policy_decimation = try std.fmt.parseInt(u32, try nextValue(it), 10);
        } else if (std.mem.eql(u8, arg, "--auto-rl")) {
            config.auto_rl = true;
        } else if (std.mem.eql(u8, arg, "--fixed-command")) {
            config.fixed_command = .{
                try std.fmt.parseFloat(f32, try nextValue(it)),
                try std.fmt.parseFloat(f32, try nextValue(it)),
                try std.fmt.parseFloat(f32, try nextValue(it)),
            };
        } else if (std.mem.eql(u8, arg, "--max-seconds")) {
            config.max_run_time_s = try std.fmt.parseFloat(f64, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--clip-actions")) {
            config.clip_actions = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kp")) {
            config.rl_gains.kp = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kd")) {
            config.rl_gains.kd = try std.fmt.parseFloat(f32, try nextValue(it));
        } else {
            std.debug.print("unknown argument: {s}\n", .{arg});
            printHelp();
            return OptionsError.InvalidValue;
        }
    }
    return config;
}

fn nextValue(it: *std.process.Args.Iterator) ![]const u8 {
    return it.next() orelse OptionsError.MissingValue;
}

fn printHelp() void {
    std.debug.print(
        \\Lite3 Zig deploy, original C++ behavior port
        \\
        \\Usage:
        \\  zig build run -- [options]
        \\
        \\Options:
        \\  --policy PATH                 default: policy/ppo/policy.onnx
        \\  --robot-ip IP                 default: 192.168.2.1
        \\  --robot-port PORT             default: 43893
        \\  --command MODE                retroid | skydroid | keyboard | none (default retroid)
        \\  --keyboard                    shortcut for --command keyboard
        \\  --no-command                  shortcut for --command none
        \\  --gamepad-port PORT           default: 12121
        \\  --decimation N                default: 12
        \\  --kp VALUE --kd VALUE          RL PD gains (default 17 / 0.9)
        \\  --clip-actions VALUE           default: 12.0
        \\  --auto-rl                     auto request StandUp then RLControl
        \\  --fixed-command VX VY WZ       normalized command in [-1, 1]
        \\  --max-seconds SEC             exit after SEC seconds
        \\
        \\Retroid: Y=stand, A=RL, left+right stick press=damping, left stick xy, right stick yaw.
        \\Keyboard (--keyboard): raw stdin when possible; z=stand, c=RL, r=damping,
        \\                     w/s/a/d/q/e adjust velocity, x or Space zeroes velocity.
        \\
    ,
        .{},
    );
}
