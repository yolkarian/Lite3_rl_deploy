const std = @import("std");
const types = @import("types.zig");
const math = @import("math.zig");

pub fn goalJointPositions() types.JointVector {
    return math.standupGoalJointPositions();
}

pub fn commandAt(
    config: types.ControllerConfig,
    start_position: types.JointVector,
    start_velocity: types.JointVector,
    elapsed_s: f64,
) types.LowLevelCommand {
    const goal = goalJointPositions();
    const duration: f32 = @floatCast(config.stand_duration_s);
    var command = types.zeroCommand();

    if (elapsed_s <= config.stand_duration_s) {
        const t: f32 = @floatCast(elapsed_s);
        for (0..types.dof_count) |index| {
            command[index] = positionCommand(
                config.swing_leg_gains,
                math.cubicPosition(start_position[index], start_velocity[index], goal[index], 0.0, t, duration),
                math.cubicVelocity(start_position[index], start_velocity[index], goal[index], 0.0, t, duration),
            );
        }
    } else {
        const t: f32 = @floatCast(elapsed_s - config.stand_duration_s);
        for (0..types.dof_count) |index| {
            command[index] = positionCommand(
                config.swing_leg_gains,
                math.cubicPosition(goal[index], 0.0, types.standup_default_joint_positions[index], 0.0, t, duration),
                math.cubicVelocity(goal[index], 0.0, types.standup_default_joint_positions[index], 0.0, t, duration),
            );
        }
    }

    return command;
}

fn positionCommand(gains: types.Gains, position: f32, velocity: f32) types.JointCommand {
    return .{
        .kp = gains.kp,
        .position = position,
        .kd = gains.kd,
        .velocity = velocity,
        .torque = 0.0,
    };
}

test "standup command uses original gains and final target" {
    const config = types.ControllerConfig{};
    const start_position = std.mem.zeroes(types.JointVector);
    const start_velocity = std.mem.zeroes(types.JointVector);
    const command = commandAt(config, start_position, start_velocity, 2.0 * config.stand_duration_s + 0.1);

    for (0..types.dof_count) |index| {
        try std.testing.expectEqual(config.swing_leg_gains.kp, command[index].kp);
        try std.testing.expectEqual(config.swing_leg_gains.kd, command[index].kd);
        try std.testing.expectApproxEqAbs(types.standup_default_joint_positions[index], command[index].position, 1.0e-6);
        try std.testing.expectApproxEqAbs(@as(f32, 0.0), command[index].velocity, 1.0e-6);
    }
}
