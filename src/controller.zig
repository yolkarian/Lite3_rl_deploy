const std = @import("std");
const types = @import("types.zig");
const math = @import("math.zig");
const motion = @import("motion.zig");
const onnx = @import("onnx_policy.zig");
const observation = @import("observation.zig");
const command_mod = @import("command.zig");

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
    stand_start_position: types.JointVector = std.mem.zeroes(types.JointVector),
    stand_start_velocity: types.JointVector = std.mem.zeroes(types.JointVector),
    last_policy_target: types.JointVector = types.default_joint_positions,
    last_policy_offset: types.JointVector = std.mem.zeroes(types.JointVector),
    last_policy_cost_ms: f64 = 0.0,
    process_start_ns: i128 = 0,

    pub fn init(allocator: std.mem.Allocator, config: types.ControllerConfig) !Controller {
        var robot = try motion.HardwareInterface.init(allocator, config.robot_ip, config.robot_port);
        errdefer robot.deinit();

        var policy = try onnx.PolicySession.init(allocator, config.policy_path, config.output_kind, config.policy_action_scale, config.clip_actions);
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
        self.robot.start();
        self.command_source.start();

        std.debug.print("Lite3 Zig deploy started. policy={s}\n", .{self.config.policy_path});
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

            const user_command = self.command_source.poll(self.state);
            try self.step(state, user_command);
        }
    }

    fn transition(self: *Controller, next_state: types.MotionState, robot_state: types.RobotState) void {
        if (next_state == self.state) return;
        std.debug.print("state {s} -> {s}\n", .{ @tagName(self.state), @tagName(next_state) });
        self.state = next_state;
        self.state_enter_time_s = robot_state.timestamp_s;
        switch (next_state) {
            .standing_up => {
                self.stand_start_position = robot_state.joint_position;
                self.stand_start_velocity = robot_state.joint_velocity;
            },
            .rl_control => {
                self.obs_builder.reset(robot_state);
                self.last_policy_target = types.default_joint_positions;
                self.last_policy_offset = std.mem.zeroes(types.JointVector);
            },
            else => {},
        }
    }

    fn step(self: *Controller, robot_state: types.RobotState, user_command: types.UserCommand) !void {
        if (user_command.target_mode == .joint_damping and self.state != .joint_damping) {
            self.transition(.joint_damping, robot_state);
        }

        switch (self.state) {
            .waiting_for_stand => try self.runIdle(robot_state, user_command),
            .standing_up => try self.runStand(robot_state, user_command),
            .rl_control => try self.runRl(robot_state, user_command),
            .joint_damping => try self.runDamping(robot_state),
        }
    }

    fn runIdle(self: *Controller, robot_state: types.RobotState, user_command: types.UserCommand) !void {
        try self.robot.send(types.zeroCommand());
        if (self.tick_count % 1000 == 0) {
            std.debug.print(
                "idle tick={} rpy(deg)=({d:.1},{d:.1},{d:.1}) q0={d:.3}\n",
                .{
                    robot_state.tick,
                    robot_state.rpy_rad[0] * types.radians_to_degrees,
                    robot_state.rpy_rad[1] * types.radians_to_degrees,
                    robot_state.rpy_rad[2] * types.radians_to_degrees,
                    robot_state.joint_position[0],
                },
            );
        }
        if (user_command.target_mode == .standing_up and self.sensorDataLooksSafe(robot_state)) {
            self.transition(.standing_up, robot_state);
        }
    }

    fn runStand(self: *Controller, robot_state: types.RobotState, user_command: types.UserCommand) !void {
        const elapsed = robot_state.timestamp_s - self.state_enter_time_s;
        var command = types.zeroCommand();
        for (0..types.dof_count) |index| {
            const pos = math.cubicPosition(
                self.stand_start_position[index],
                self.stand_start_velocity[index],
                types.default_joint_positions[index],
                0.0,
                @floatCast(elapsed),
                @floatCast(self.config.stand_duration_s),
            );
            const vel = math.cubicVelocity(
                self.stand_start_position[index],
                self.stand_start_velocity[index],
                types.default_joint_positions[index],
                0.0,
                @floatCast(elapsed),
                @floatCast(self.config.stand_duration_s),
            );
            command[index] = .{
                .kp = self.config.stand_gains.kp,
                .position = pos,
                .kd = self.config.stand_gains.kd,
                .velocity = vel,
                .torque = 0.0,
            };
        }
        try self.robot.send(command);

        if (elapsed >= self.config.stand_duration_s and user_command.target_mode == .rl_control) {
            self.transition(.rl_control, robot_state);
        }
    }

    fn runRl(self: *Controller, robot_state: types.RobotState, user_command: types.UserCommand) !void {
        if (postureUnsafe(robot_state)) {
            std.debug.print("unsafe posture: roll={d:.1} pitch={d:.1}; entering damping\n", .{
                robot_state.rpy_rad[0] * types.radians_to_degrees,
                robot_state.rpy_rad[1] * types.radians_to_degrees,
            });
            self.transition(.joint_damping, robot_state);
            try self.runDamping(robot_state);
            return;
        }

        var command_for_policy = user_command;
        command_for_policy.linear_x *= self.config.max_command[0];
        command_for_policy.linear_y *= self.config.max_command[1];
        command_for_policy.yaw_rate *= self.config.max_command[2];

        if (self.tick_count % self.config.policy_decimation == 0) {
            var raw_obs = self.obs_builder.appendForInference(robot_state, command_for_policy);
            const start_ns = monotonicNano();
            self.last_policy_target = try self.policy.run(&raw_obs, &self.obs_builder.obs_history);
            const end_ns = monotonicNano();
            self.last_policy_cost_ms = @as(f64, @floatFromInt(end_ns - start_ns)) / @as(f64, std.time.ns_per_ms);

            for (0..types.dof_count) |index| {
                self.last_policy_offset[index] = self.last_policy_target[index] - types.default_joint_positions[index];
            }
            self.obs_builder.updateAfterPolicy(robot_state, self.last_policy_offset);
        }

        var low_level = types.zeroCommand();
        for (0..types.dof_count) |index| {
            low_level[index] = .{
                .kp = self.config.rl_gains.kp,
                .position = self.last_policy_target[index],
                .kd = self.config.rl_gains.kd,
                .velocity = 0.0,
                .torque = 0.0,
            };
        }
        try self.robot.send(low_level);

        if (self.tick_count % 1000 == 0) {
            std.debug.print("rl tick={} cmd=({d:.2},{d:.2},{d:.2}) ort={d:.3}ms qdes0={d:.3}\n", .{
                robot_state.tick,
                command_for_policy.linear_x,
                command_for_policy.linear_y,
                command_for_policy.yaw_rate,
                self.last_policy_cost_ms,
                self.last_policy_target[0],
            });
        }
    }

    fn runDamping(self: *Controller, robot_state: types.RobotState) !void {
        var command = types.zeroCommand();
        for (0..types.dof_count) |index| {
            command[index].kp = self.config.damping_gains.kp;
            command[index].kd = self.config.damping_gains.kd;
        }
        try self.robot.send(command);

        if (robot_state.timestamp_s - self.state_enter_time_s >= self.config.damping_duration_s) {
            self.transition(.waiting_for_stand, robot_state);
        }
    }

    fn sensorDataLooksSafe(self: *Controller, robot_state: types.RobotState) bool {
        _ = self;
        if (postureUnsafe(robot_state)) return false;
        const acc_norm = norm3(robot_state.linear_acc_m_s2);
        if (acc_norm < 0.1 * types.gravity or acc_norm > 3.0 * types.gravity) return false;
        for (robot_state.joint_position) |value| {
            if (std.math.isNan(value)) return false;
        }
        return true;
    }
};

fn postureUnsafe(robot_state: types.RobotState) bool {
    return @abs(robot_state.rpy_rad[0]) > 30.0 * types.degrees_to_radians or
        @abs(robot_state.rpy_rad[1]) > 45.0 * types.degrees_to_radians;
}

fn norm3(value: types.Vec3) f32 {
    return @sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
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
