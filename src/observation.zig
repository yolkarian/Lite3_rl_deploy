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

        var initial_offset: types.JointVector = undefined;
        for (0..types.dof_count) |i| {
            initial_offset[i] = state.joint_position[i] - types.default_joint_positions[i];
        }
        for (0..types.action_history_steps) |i| self.action_history[i] = initial_offset;

        // Match legged-training: the 40-step observation history buffer is
        // zero-initialized at episode reset (ObservationHistoryBuffer uses
        // torch.zeros, reset_ids zeros per-env, eval calls write_full(0.0)).
        // The first policy call must see [0 x 39, obs_0], not [obs_0 x 40].
        self.obs_history = std.mem.zeroes(types.RawObservationHistory);
        self.initialized = true;
    }

    pub fn makeRawObservation(self: *const ObservationBuilder, state: types.RobotState, command: types.UserCommand) types.RawObservation {
        var obs: types.RawObservation = undefined;
        var cursor: usize = 0;

        obs[cursor] = command.linear_x;
        obs[cursor + 1] = command.linear_y;
        obs[cursor + 2] = command.yaw_rate;
        cursor += 3;

        inline for (0..3) |i| obs[cursor + i] = state.rpy_rad[i];
        cursor += 3;

        inline for (0..3) |i| obs[cursor + i] = state.angular_velocity_rad_s[i];
        cursor += 3;

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

    pub fn appendForInference(self: *ObservationBuilder, state: types.RobotState, command: types.UserCommand) types.RawObservation {
        if (!self.initialized) self.reset(state);
        const obs = self.makeRawObservation(state, command);
        rollObservationHistory(&self.obs_history);
        self.obs_history[types.obs_history_horizon - 1] = obs;
        return obs;
    }

    pub fn updateAfterPolicy(self: *ObservationBuilder, state: types.RobotState, action_offset: types.JointVector) void {
        rollJointHistory(types.position_history_steps, &self.position_history);
        self.position_history[types.position_history_steps - 1] = state.joint_position;

        rollJointHistory(types.velocity_history_steps, &self.velocity_history);
        self.velocity_history[types.velocity_history_steps - 1] = state.joint_velocity;

        rollJointHistory(types.action_history_steps, &self.action_history);
        self.action_history[types.action_history_steps - 1] = action_offset;
    }
};

fn rollJointHistory(comptime len: usize, history: *[len]types.JointVector) void {
    if (len <= 1) return;
    for (0..len - 1) |i| history[i] = history[i + 1];
}

fn rollObservationHistory(history: *types.RawObservationHistory) void {
    for (0..types.obs_history_horizon - 1) |i| history[i] = history[i + 1];
}

test "raw observation layout dimension" {
    var builder = ObservationBuilder{};
    var state = types.RobotState{};
    state.joint_position = types.default_joint_positions;
    builder.reset(state);
    const obs = builder.makeRawObservation(state, .{ .linear_x = 0.1, .linear_y = -0.2, .yaw_rate = 0.3 });
    try std.testing.expectEqual(@as(f32, 0.1), obs[0]);
    try std.testing.expectEqual(@as(f32, -0.2), obs[1]);
    try std.testing.expectEqual(@as(f32, 0.3), obs[2]);
    try std.testing.expectEqual(@as(f32, types.default_joint_positions[0]), obs[9]);
}

test "reset zeroes obs_history; first inference sees [0 x 39, obs_0]" {
    var builder = ObservationBuilder{};
    var state = types.RobotState{};
    state.joint_position = types.default_joint_positions;
    builder.reset(state);

    // After reset the 40-step history buffer is all zeros.
    for (builder.obs_history[0 .. types.obs_history_horizon - 1]) |entry| {
        try std.testing.expectEqual(@as(f32, 0.0), entry[0]);
    }

    // First inference call rolls zeros left and writes obs_0 at the last slot.
    const first_obs = builder.appendForInference(state, .{ .linear_x = 0.5 });
    try std.testing.expectEqual(@as(f32, 0.5), builder.obs_history[types.obs_history_horizon - 1][0]);
    try std.testing.expectEqual(@as(f32, 0.0), builder.obs_history[0][0]);
    try std.testing.expectEqual(first_obs[0], builder.obs_history[types.obs_history_horizon - 1][0]);
}
