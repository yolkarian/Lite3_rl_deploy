const std = @import("std");
const types = @import("types.zig");
const motion = @import("motion.zig");
const onnx = @import("onnx_policy.zig");
const observation = @import("observation.zig");
const command_mod = @import("command.zig");
const standup = @import("standup.zig");

pub const Controller = struct {
    allocator: std.mem.Allocator,
    config: types.ControllerConfig,
    robot: motion.HardwareInterface,
    policy: onnx.PolicySession,
    command_source: command_mod.CommandSource,
    obs_builder: observation.ObservationBuilder = .{},
    state: types.MotionState = .waiting_for_stand,
    last_tick: u32 = 0,
    tick_count: u64 = 0,
    state_enter_time_s: f64 = 0.0,
    idle_first_enter: bool = true,
    idle_last_print_time_s: f64 = -10000.0,
    stand_start_position: types.JointVector = std.mem.zeroes(types.JointVector),
    stand_start_velocity: types.JointVector = std.mem.zeroes(types.JointVector),
    last_policy_target: types.JointVector = types.policy_default_joint_positions,
    last_policy_offset: types.JointVector = std.mem.zeroes(types.JointVector),
    last_policy_cost_ms: f64 = 0.0,
    rl_run_count: i64 = -1,
    process_start_ns: i128 = 0,
    freq_window_start_ns: i128 = 0,
    state_window_count: u64 = 0,
    send_window_count: u64 = 0,
    policy_window_count: u64 = 0,

    pub fn init(allocator: std.mem.Allocator, config: types.ControllerConfig) !Controller {
        var robot = try motion.HardwareInterface.init(allocator, config.robot_ip, config.robot_port);
        errdefer robot.deinit();

        var policy = try onnx.PolicySession.init(allocator, config.policy_path, config.clip_actions);
        errdefer policy.deinit();

        return .{
            .allocator = allocator,
            .config = config,
            .robot = robot,
            .policy = policy,
            .command_source = command_mod.CommandSource.init(config),
        };
    }

    pub fn deinit(self: *Controller) void {
        self.command_source.stop();
        self.policy.deinit();
        self.robot.deinit();
    }

    pub fn run(self: *Controller) !void {
        self.process_start_ns = monotonicNano();
        self.freq_window_start_ns = self.process_start_ns;
        self.robot.start();
        self.command_source.start();

        std.debug.print("Lite3 Zig deploy (original-compatible) started. policy={s}\n", .{self.config.policy_path});
        while (true) {
            const state = self.robot.readState() catch {
                sleepNs(500 * std.time.ns_per_us);
                continue;
            };

            if (self.config.max_run_time_s) |limit_s| {
                const elapsed_s = @as(f64, @floatFromInt(monotonicNano() - self.process_start_ns)) / @as(f64, std.time.ns_per_s);
                if (elapsed_s >= limit_s) break;
            }

            if (state.tick == self.last_tick) {
                sleepNs(500 * std.time.ns_per_us);
                continue;
            }
            self.last_tick = state.tick;
            self.tick_count += 1;
            if (self.tick_count == 1 and self.state == .waiting_for_stand) {
                self.state_enter_time_s = state.timestamp_s;
                std.debug.print("Waiting for stand up...\n", .{});
            }

            const user_command = self.command_source.poll(self.state);
            try self.step(state, user_command);
            self.printFrequencies();
        }
    }

    fn transition(self: *Controller, next_state: types.MotionState, robot_state: types.RobotState) void {
        if (next_state == self.state) return;
        std.debug.print("\n{s} ------------> {s}\n", .{ stateName(self.state), stateName(next_state) });
        self.state = next_state;
        self.state_enter_time_s = robot_state.timestamp_s;
        switch (next_state) {
            .waiting_for_stand => {
                self.idle_last_print_time_s = -10000.0;
                std.debug.print("Waiting for stand up...\n", .{});
            },
            .standing_up => {
                self.stand_start_position = robot_state.joint_position;
                self.stand_start_velocity = robot_state.joint_velocity;
            },
            .rl_control => {
                self.rl_run_count = -1;
                self.obs_builder = .{};
                self.last_policy_target = types.policy_default_joint_positions;
                self.last_policy_offset = std.mem.zeroes(types.JointVector);
                self.last_policy_cost_ms = 0.0;
                std.debug.print("[ONNX ENTER] PolicyRunner entered: test_onnx\n", .{});
            },
            .joint_damping => {},
        }
    }

    fn step(self: *Controller, robot_state: types.RobotState, user_command: types.UserCommand) !void {
        switch (self.state) {
            .waiting_for_stand => try self.runIdle(robot_state, user_command),
            .standing_up => try self.runStand(robot_state, user_command),
            .rl_control => try self.runRl(robot_state, user_command),
            .joint_damping => try self.runDamping(robot_state),
        }
    }

    fn runIdle(self: *Controller, robot_state: types.RobotState, user_command: types.UserCommand) !void {
        const joint_normal = jointDataNormalCheck(robot_state);
        const imu_normal = imuDataNormalCheck(robot_state);

        if (robot_state.timestamp_s - self.idle_last_print_time_s > 1.0) {
            displayProprioceptiveInfo(robot_state);
            displayAxisValue(user_command);
            self.idle_last_print_time_s = robot_state.timestamp_s;
        }

        try self.sendCommand(types.zeroCommand());

        if (!joint_normal or !imu_normal) {
            if (self.tick_count % 1000 == 0) std.debug.print("joint status: {} | imu status: {}\n", .{ joint_normal, imu_normal });
            return;
        }
        if (self.idle_first_enter and robot_state.timestamp_s - self.state_enter_time_s < 0.5) return;

        if (user_command.target_mode == .standing_up) {
            self.transition(.standing_up, robot_state);
            self.idle_first_enter = false;
        }
    }

    fn runStand(self: *Controller, robot_state: types.RobotState, user_command: types.UserCommand) !void {
        const elapsed = robot_state.timestamp_s - self.state_enter_time_s;
        const command = standup.commandAt(self.config, self.stand_start_position, self.stand_start_velocity, elapsed);
        try self.sendCommand(command);

        if (user_command.target_mode == .joint_damping) {
            self.transition(.joint_damping, robot_state);
            return;
        }

        if (elapsed > 2.0 * self.config.stand_duration_s and user_command.target_mode == .rl_control) {
            self.transition(.rl_control, robot_state);
        }
    }

    fn runRl(self: *Controller, robot_state: types.RobotState, user_command: types.UserCommand) !void {
        if (user_command.target_mode == .joint_damping or postureUnsafe(robot_state)) {
            if (postureUnsafe(robot_state)) std.debug.print("posture value: {d:.1} {d:.1} {d:.1}\n", .{
                robot_state.rpy_rad[0] * types.radians_to_degrees,
                robot_state.rpy_rad[1] * types.radians_to_degrees,
                robot_state.rpy_rad[2] * types.radians_to_degrees,
            });
            self.transition(.joint_damping, robot_state);
            try self.runDamping(robot_state);
            return;
        }

        self.applyGainTuning(user_command);

        self.rl_run_count += 1;
        if (@mod(self.rl_run_count, @as(i64, @intCast(self.config.policy_decimation))) != 0) return;

        if (!self.obs_builder.initialized) {
            self.obs_builder.reset(robot_state);
            self.last_policy_offset = types.jointOffsetFromPolicyDefault(robot_state.joint_position);
        }
        var raw_obs = self.obs_builder.appendForInference(robot_state, user_command, self.config.max_cmd_vel);
        const previous_policy_offset = self.last_policy_offset;
        self.obs_builder.advanceAfterObservation(robot_state, previous_policy_offset);

        const start_ns = monotonicNano();
        self.last_policy_target = try self.policy.run(&raw_obs, &self.obs_builder.obs_history);
        self.policy_window_count += 1;
        const end_ns = monotonicNano();
        self.last_policy_cost_ms = @as(f64, @floatFromInt(end_ns - start_ns)) / @as(f64, std.time.ns_per_ms);

        for (0..types.dof_count) |index| {
            self.last_policy_offset[index] = self.last_policy_target[index] - types.policy_default_joint_positions[index];
        }

        var low_level = types.zeroCommand();
        for (0..types.dof_count) |index| low_level[index] = positionCommand(self.config.rl_gains, self.last_policy_target[index], 0.0);
        try self.sendCommand(low_level);

        if (@mod(self.rl_run_count, 1000) == 0) {
            std.debug.print("\nrl tick={} cmd=({d:.2},{d:.2},{d:.2}) ort={d:.3}ms qdes0={d:.3}\n", .{
                robot_state.tick,
                user_command.forward_vel_scale,
                user_command.side_vel_scale,
                user_command.turnning_vel_scale,
                self.last_policy_cost_ms,
                self.last_policy_target[0],
            });
        }
    }

    fn applyGainTuning(self: *Controller, user_command: types.UserCommand) void {
        const has_delta = user_command.rl_kp_delta != 0.0 or user_command.rl_kd_delta != 0.0;
        if (!has_delta and !user_command.print_gains) return;

        if (user_command.rl_kp_delta != 0.0) {
            self.config.rl_gains.kp = types.clamp(self.config.rl_gains.kp + user_command.rl_kp_delta, self.config.gain_kp_min, self.config.gain_kp_max);
        }
        if (user_command.rl_kd_delta != 0.0) {
            self.config.rl_gains.kd = types.clamp(self.config.rl_gains.kd + user_command.rl_kd_delta, self.config.gain_kd_min, self.config.gain_kd_max);
        }

        std.debug.print("\n[gain-tune] Kp={d:.3} Kd={d:.3}\n", .{ self.config.rl_gains.kp, self.config.rl_gains.kd });
    }

    fn runDamping(self: *Controller, robot_state: types.RobotState) !void {
        var command = types.zeroCommand();
        for (0..types.dof_count) |index| {
            command[index].kp = self.config.damping_gains.kp;
            command[index].kd = self.config.damping_gains.kd;
        }
        try self.sendCommand(command);

        if (robot_state.timestamp_s - self.state_enter_time_s >= self.config.damping_duration_s) {
            self.transition(.waiting_for_stand, robot_state);
        }
    }

    fn sendCommand(self: *Controller, command: types.LowLevelCommand) !void {
        try self.robot.send(command);
        self.send_window_count += 1;
    }

    fn printFrequencies(self: *Controller) void {
        self.state_window_count += 1;
        const now = monotonicNano();
        const elapsed_s = @as(f64, @floatFromInt(now - self.freq_window_start_ns)) / @as(f64, std.time.ns_per_s);
        if (elapsed_s >= 1.0) {
            const state_hz = @as(f64, @floatFromInt(self.state_window_count)) / elapsed_s;
            const send_hz = @as(f64, @floatFromInt(self.send_window_count)) / elapsed_s;
            const policy_hz = @as(f64, @floatFromInt(self.policy_window_count)) / elapsed_s;
            std.debug.print("\r[freq] rx={d:.1}Hz send={d:.1}Hz policy={d:.1}Hz mode={s} decim={}\x1b[K", .{
                state_hz,
                send_hz,
                policy_hz,
                stateName(self.state),
                self.config.policy_decimation,
            });
            self.freq_window_start_ns = now;
            self.state_window_count = 0;
            self.send_window_count = 0;
            self.policy_window_count = 0;
        }
    }
};

fn positionCommand(gains: types.Gains, position: f32, velocity: f32) types.JointCommand {
    return .{
        .kp = gains.kp,
        .position = position,
        .kd = gains.kd,
        .velocity = velocity,
        .torque = 0.0,
    };
}

fn jointDataNormalCheck(robot_state: types.RobotState) bool {
    for (0..types.dof_count) |index| {
        const lower = jointLower(index);
        const upper = jointUpper(index);
        if (std.math.isNan(robot_state.joint_position[index]) or robot_state.joint_position[index] > upper + 0.1 or robot_state.joint_position[index] < lower - 0.1) return false;
        if (std.math.isNan(robot_state.joint_velocity[index]) or robot_state.joint_velocity[index] > types.joint_velocity_limit[index % 3] + 0.1) return false;
    }
    return true;
}

fn imuDataNormalCheck(robot_state: types.RobotState) bool {
    for (0..3) |index| {
        if (std.math.isNan(robot_state.rpy_rad[index]) or @abs(robot_state.rpy_rad[index]) > std.math.pi) return false;
        if (std.math.isNan(robot_state.angular_velocity[index]) or @abs(robot_state.angular_velocity[index]) > std.math.pi) return false;
    }
    const acc_norm = norm3(robot_state.linear_acc_m_s2);
    if (acc_norm < 0.1 * types.gravity or acc_norm > 3.0 * types.gravity) return false;
    return true;
}

fn jointLower(index: usize) f32 {
    const local = index % 3;
    if (local == 0 and (index / 3 == 1 or index / 3 == 3)) return -types.fl_joint_upper[0];
    return types.fl_joint_lower[local];
}

fn jointUpper(index: usize) f32 {
    const local = index % 3;
    if (local == 0 and (index / 3 == 1 or index / 3 == 3)) return -types.fl_joint_lower[0];
    return types.fl_joint_upper[local];
}

fn postureUnsafe(robot_state: types.RobotState) bool {
    return @abs(robot_state.rpy_rad[0]) > 30.0 * types.degrees_to_radians or
        @abs(robot_state.rpy_rad[1]) > 45.0 * types.degrees_to_radians;
}

fn norm3(value: types.Vec3) f32 {
    return @sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

fn displayProprioceptiveInfo(robot_state: types.RobotState) void {
    std.debug.print("\nJoint Data:\n", .{});
    printJointVector("pos", robot_state.joint_position);
    printJointVector("vel", robot_state.joint_velocity);
    printJointVector("tau", robot_state.joint_torque);
    std.debug.print("Imu Data:\n", .{});
    printVec3("rpy", robot_state.rpy_rad);
    printVec3("acc", robot_state.linear_acc_m_s2);
    printVec3("omg", robot_state.angular_velocity);
}

fn displayAxisValue(command: types.UserCommand) void {
    std.debug.print("User Command Input:\n", .{});
    std.debug.print("axis value:  {d:.3} {d:.3} {d:.3}\n", .{ command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale });
    std.debug.print("target mode: {}\n", .{@intFromEnum(command.target_mode)});
}

fn printJointVector(label: []const u8, value: types.JointVector) void {
    std.debug.print("{s}:", .{label});
    for (value) |item| std.debug.print(" {d:.4}", .{item});
    std.debug.print("\n", .{});
}

fn printVec3(label: []const u8, value: types.Vec3) void {
    std.debug.print("{s}: {d:.4} {d:.4} {d:.4}\n", .{ label, value[0], value[1], value[2] });
}

fn stateName(state: types.MotionState) []const u8 {
    return switch (state) {
        .waiting_for_stand => "idle_state",
        .standing_up => "standup_state",
        .rl_control => "rl_control",
        .joint_damping => "joint_damping",
    };
}

fn monotonicNano() i128 {
    var ts: std.c.timespec = undefined;
    _ = std.c.clock_gettime(std.c.CLOCK.MONOTONIC, &ts);
    return @as(i128, ts.sec) * std.time.ns_per_s + @as(i128, ts.nsec);
}

fn sleepNs(ns: u64) void {
    var request = std.c.timespec{
        .sec = @intCast(ns / std.time.ns_per_s),
        .nsec = @intCast(ns % std.time.ns_per_s),
    };
    while (std.c.nanosleep(&request, &request) != 0) {}
}
