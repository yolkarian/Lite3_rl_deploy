const std = @import("std");
const types = @import("types.zig");

pub const ObservationBuilder = struct {
    position_history: [types.position_history_steps]types.JointVector = std.mem.zeroes([types.position_history_steps]types.JointVector),
    velocity_history: [types.velocity_history_steps]types.JointVector = std.mem.zeroes([types.velocity_history_steps]types.JointVector),
    action_history: [types.action_history_steps]types.JointVector = std.mem.zeroes([types.action_history_steps]types.JointVector),
    obs_history: types.RawObservationHistory = std.mem.zeroes(types.RawObservationHistory),
    initialized: bool = false,

    pub fn reset(self: *ObservationBuilder, state: types.RobotState) void {
        for (0..types.position_history_steps) |i| self.position_history[i] = state.joint_position;
        for (0..types.velocity_history_steps) |i| self.velocity_history[i] = state.joint_velocity;

        const initial_offset = types.jointOffsetFromPolicyDefault(state.joint_position);
        for (0..types.action_history_steps) |i| self.action_history[i] = initial_offset;

        self.obs_history = std.mem.zeroes(types.RawObservationHistory);
        self.initialized = true;
    }

    pub fn makeRawObservation(
        self: *const ObservationBuilder,
        state: types.RobotState,
        command: types.UserCommand,
        max_cmd_vel: types.Vec3,
    ) types.RawObservation {
        var obs: types.RawObservation = undefined;
        var cursor: usize = 0;

        obs[cursor] = command.forward_vel_scale * max_cmd_vel[0];
        obs[cursor + 1] = command.side_vel_scale * max_cmd_vel[1];
        obs[cursor + 2] = command.turnning_vel_scale * max_cmd_vel[2];
        cursor += 3;

        inline for (0..3) |i| obs[cursor + i] = state.rpy_rad[i];
        cursor += 3;

        inline for (0..3) |i| obs[cursor + i] = state.angular_velocity[i];
        cursor += 3;

        // Original C++ ONNX runner feeds absolute joint angles here.  The ONNX
        // graph contains the normalization expected by the policy export.
        for (0..types.dof_count) |i| obs[cursor + i] = state.joint_position[i];
        cursor += types.dof_count;

        for (0..types.dof_count) |i| obs[cursor + i] = state.joint_velocity[i];
        cursor += types.dof_count;

        for (self.position_history) |history_item| {
            for (0..types.dof_count) |i| obs[cursor + i] = history_item[i];
            cursor += types.dof_count;
        }

        for (self.velocity_history) |history_item| {
            for (0..types.dof_count) |i| obs[cursor + i] = history_item[i];
            cursor += types.dof_count;
        }

        for (self.action_history) |history_item| {
            for (0..types.dof_count) |i| obs[cursor + i] = history_item[i];
            cursor += types.dof_count;
        }

        comptime std.debug.assert(types.raw_obs_dim == 117);
        std.debug.assert(cursor == types.raw_obs_dim);
        return obs;
    }

    pub fn appendForInference(
        self: *ObservationBuilder,
        state: types.RobotState,
        command: types.UserCommand,
        max_cmd_vel: types.Vec3,
    ) types.RawObservation {
        if (!self.initialized) self.reset(state);
        const obs = self.makeRawObservation(state, command, max_cmd_vel);
        rollObservationHistory(&self.obs_history);
        self.obs_history[types.obs_history_horizon - 1] = obs;
        return obs;
    }

    pub fn advanceAfterObservation(self: *ObservationBuilder, state: types.RobotState, previous_action_offset_policy: types.JointVector) void {
        rollJointHistory(types.position_history_steps, &self.position_history);
        self.position_history[types.position_history_steps - 1] = state.joint_position;

        rollJointHistory(types.velocity_history_steps, &self.velocity_history);
        self.velocity_history[types.velocity_history_steps - 1] = state.joint_velocity;

        rollJointHistory(types.action_history_steps, &self.action_history);
        self.action_history[types.action_history_steps - 1] = previous_action_offset_policy;
    }
};

fn rollJointHistory(comptime len: usize, history: *[len]types.JointVector) void {
    if (len <= 1) return;
    for (0..len - 1) |i| history[i] = history[i + 1];
}

fn rollObservationHistory(history: *types.RawObservationHistory) void {
    for (0..types.obs_history_horizon - 1) |i| history[i] = history[i + 1];
}

test "raw observation layout matches original ONNX runner" {
    var builder = ObservationBuilder{};
    var state = types.RobotState{};
    state.joint_position = types.policy_default_joint_positions;
    state.joint_velocity[0] = 0.2;
    state.rpy_rad = .{ 0.1, 0.2, 0.3 };
    state.angular_velocity = .{ 1.0, 2.0, 3.0 };
    builder.reset(state);

    const obs = builder.makeRawObservation(state, .{ .forward_vel_scale = 0.5, .side_vel_scale = -0.25, .turnning_vel_scale = 1.0 }, .{ 0.8, 0.8, 0.8 });
    try std.testing.expectEqual(@as(f32, 0.4), obs[0]);
    try std.testing.expectEqual(@as(f32, -0.2), obs[1]);
    try std.testing.expectEqual(@as(f32, 0.8), obs[2]);
    try std.testing.expectEqual(@as(f32, 0.1), obs[3]);
    try std.testing.expectEqual(@as(f32, 1.0), obs[6]);
    try std.testing.expectEqual(types.policy_default_joint_positions[0], obs[9]);
    try std.testing.expectEqual(types.policy_default_joint_positions[1], obs[10]);
    try std.testing.expectEqual(@as(f32, 0.2), obs[21]);
    try std.testing.expectEqual(types.policy_default_joint_positions[0], obs[33]);
    try std.testing.expectEqual(@as(f32, 0.0), obs[93]);
}

test "reset zeroes 40-step raw observation history" {
    var builder = ObservationBuilder{};
    var state = types.RobotState{};
    state.joint_position = types.policy_default_joint_positions;
    builder.reset(state);

    for (builder.obs_history) |entry| try std.testing.expectEqual(@as(f32, 0.0), entry[0]);

    const first_obs = builder.appendForInference(state, .{}, .{ 0.8, 0.8, 0.8 });
    try std.testing.expectEqual(first_obs[0], builder.obs_history[types.obs_history_horizon - 1][0]);
    try std.testing.expectEqual(@as(f32, 0.0), builder.obs_history[0][0]);
}

test "short histories advance only after observation build" {
    var builder = ObservationBuilder{};
    var state0 = types.RobotState{};
    state0.joint_position = types.policy_default_joint_positions;
    builder.reset(state0);

    const zero_offset = std.mem.zeroes(types.JointVector);
    var action0 = std.mem.zeroes(types.JointVector);
    action0[0] = 0.25;

    const obs0 = builder.appendForInference(state0, .{}, .{ 0.8, 0.8, 0.8 });
    try std.testing.expectEqual(types.policy_default_joint_positions[0], obs0[33]);
    try std.testing.expectEqual(@as(f32, 0.0), obs0[93]);
    builder.advanceAfterObservation(state0, zero_offset);

    var state1 = state0;
    state1.joint_position[0] = 0.1;
    state1.joint_velocity[0] = 0.2;
    const obs1 = builder.appendForInference(state1, .{}, .{ 0.8, 0.8, 0.8 });
    try std.testing.expectEqual(types.policy_default_joint_positions[0], obs1[33]);
    try std.testing.expectEqual(@as(f32, 0.0), obs1[93]);
    builder.advanceAfterObservation(state1, action0);

    var state2 = state1;
    state2.joint_position[0] = 0.2;
    const obs2 = builder.appendForInference(state2, .{}, .{ 0.8, 0.8, 0.8 });
    try std.testing.expectEqual(types.policy_default_joint_positions[0], obs2[33]);
    try std.testing.expectEqual(types.policy_default_joint_positions[0], obs2[45]);
    try std.testing.expectEqual(@as(f32, 0.1), obs2[57]);
    try std.testing.expectEqual(@as(f32, 0.25), obs2[105]);
}
