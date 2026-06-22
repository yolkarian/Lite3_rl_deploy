const std = @import("std");
const deploy = @import("lite3_deploy");

const OptionsError = error{ MissingValue, InvalidValue };

pub fn main(init: std.process.Init) !void {
    var it = try std.process.Args.Iterator.initAllocator(init.minimal.args, init.gpa);
    defer it.deinit();

    var config = try parseArgs(&it);
    if (config.policy_path.len == 0) config.policy_path = "policy/deploy/lite3_policy.onnx";

    var controller = try deploy.Controller.init(init.gpa, config);
    defer controller.deinit();
    try controller.run();
}

fn parseArgs(it: *std.process.Args.Iterator) !deploy.types.ControllerConfig {
    var config = deploy.types.ControllerConfig{
        .policy_path = "policy/deploy/lite3_policy.onnx",
    };

    _ = it.skip(); // executable name
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
        } else if (std.mem.eql(u8, arg, "--output-kind")) {
            config.output_kind = deploy.types.PolicyOutputKind.parse(try nextValue(it)) orelse return OptionsError.InvalidValue;
        } else if (std.mem.eql(u8, arg, "--decimation")) {
            config.policy_decimation = try std.fmt.parseInt(u32, try nextValue(it), 10);
        } else if (std.mem.eql(u8, arg, "--auto-rl")) {
            config.auto_rl = true;
        } else if (std.mem.eql(u8, arg, "--no-keyboard")) {
            config.keyboard = false;
        } else if (std.mem.eql(u8, arg, "--fixed-command")) {
            config.fixed_command = .{
                try std.fmt.parseFloat(f32, try nextValue(it)),
                try std.fmt.parseFloat(f32, try nextValue(it)),
                try std.fmt.parseFloat(f32, try nextValue(it)),
            };
        } else if (std.mem.eql(u8, arg, "--max-seconds")) {
            config.max_run_time_s = try std.fmt.parseFloat(f64, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kp")) {
            config.rl_gains.kp = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--kd")) {
            config.rl_gains.kd = try std.fmt.parseFloat(f32, try nextValue(it));
        } else if (std.mem.eql(u8, arg, "--clip-actions")) {
            config.clip_actions = try std.fmt.parseFloat(f32, try nextValue(it));
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
        \\Lite3 Zig deploy
        \\
        \\Usage:
        \\  zig build run -- --policy policy/deploy/lite3_policy.onnx [options]
        \\
        \\Options:
        \\  --policy PATH                 legged-training exported ONNX path
        \\  --robot-ip IP                 Lite3 robot IP (default 192.168.2.1)
        \\  --robot-port PORT             Lite3 SDK command port (default 43893)
        \\  --output-kind KIND            joint-target | action-offset | policy-action
        \\  --decimation N                policy period in SDK ticks (default 12)
        \\  --kp VALUE --kd VALUE          RL PD gains (default 20 / 0.7)
        \\  --clip-actions VALUE           policy-action clip bound (default 12.0)
        \\  --auto-rl                     automatically StandUp then RLControl
        \\  --fixed-command VX VY WZ       normalized command, usually in [-1, 1]
        \\  --no-keyboard                 disable stdin keyboard polling
        \\  --max-seconds SEC             exit after SEC seconds (for smoke tests)
        \\
        \\Keyboard: z=stand, c=RL, r=damping, wasd=xy, q/e=yaw, x=zero velocity.
        \\
    ,
        .{},
    );
}
