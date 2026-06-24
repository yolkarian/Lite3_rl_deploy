const std = @import("std");
const types = @import("types.zig");

pub const Mat3 = [3][3]f32;

pub fn rpyToMatrix(rpy: types.Vec3) Mat3 {
    const roll = rpy[0];
    const pitch = rpy[1];
    const yaw = rpy[2];

    const cr = @cos(roll);
    const sr = @sin(roll);
    const cp = @cos(pitch);
    const sp = @sin(pitch);
    const cy = @cos(yaw);
    const sy = @sin(yaw);

    return .{
        .{ cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
        .{ sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
        .{ -sp, cp * sr, cp * cr },
    };
}

pub fn transposeMultiplyVec3(matrix: Mat3, vector: types.Vec3) types.Vec3 {
    return .{
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2],
    };
}

pub fn projectedGravity(rpy: types.Vec3) types.Vec3 {
    return transposeMultiplyVec3(rpyToMatrix(rpy), .{ 0.0, 0.0, -1.0 });
}

pub fn cubicPosition(x0: f32, v0: f32, xf: f32, vf: f32, t: f32, duration: f32) f32 {
    if (duration <= 0.0 or t >= duration) return xf;
    if (t <= 0.0) return x0;
    const duration2 = duration * duration;
    const duration3 = duration2 * duration;
    const t2 = t * t;
    const t3 = t2 * t;
    const a = (vf * duration - 2.0 * xf + v0 * duration + 2.0 * x0) / duration3;
    const b = (3.0 * xf - vf * duration - 2.0 * v0 * duration - 3.0 * x0) / duration2;
    return a * t3 + b * t2 + v0 * t + x0;
}

pub fn cubicVelocity(x0: f32, v0: f32, xf: f32, vf: f32, t: f32, duration: f32) f32 {
    if (duration <= 0.0 or t >= duration) return vf;
    if (t <= 0.0) return v0;
    const duration2 = duration * duration;
    const duration3 = duration2 * duration;
    const t2 = t * t;
    const a = (vf * duration - 2.0 * xf + v0 * duration + 2.0 * x0) / duration3;
    const b = (3.0 * xf - vf * duration - 2.0 * v0 * duration - 3.0 * x0) / duration2;
    return 3.0 * a * t2 + 2.0 * b * t + v0;
}

pub fn standupGoalJointPositions() types.JointVector {
    const hip_y = hipYByHeight(types.pre_height);
    const knee = kneeByHeight(types.pre_height);
    return .{
        0.0, hip_y, knee,
        0.0, hip_y, knee,
        0.0, hip_y, knee,
        0.0, hip_y, knee,
    };
}

pub fn hipYByHeight(height: f32) f32 {
    const l1 = types.thigh_len;
    const l2 = types.shank_len;
    if (@abs(height) >= l1 + l2) return 0.0;
    const cos_arg = (l1 * l1 + height * height - l2 * l2) / (2.0 * height * l1);
    const theta = -std.math.acos(clampUnit(cos_arg));
    return types.clamp(theta, types.fl_joint_lower[1], types.fl_joint_upper[1]);
}

pub fn kneeByHeight(height: f32) f32 {
    const l1 = types.thigh_len;
    const l2 = types.shank_len;
    if (@abs(height) >= l1 + l2) return 0.0;
    const cos_arg = (l1 * l1 + l2 * l2 - height * height) / (2.0 * l1 * l2);
    const theta = std.math.pi - std.math.acos(clampUnit(cos_arg));
    return types.clamp(theta, types.fl_joint_lower[2], types.fl_joint_upper[2]);
}

fn clampUnit(value: f32) f32 {
    return types.clamp(value, -1.0, 1.0);
}

test "projected gravity is world down at zero rpy" {
    const g = projectedGravity(.{ 0.0, 0.0, 0.0 });
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), g[0], 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), g[1], 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), g[2], 1.0e-6);
}

test "cubic reaches endpoints" {
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), cubicPosition(1.0, 0.0, 2.0, 0.0, 0.0, 2.0), 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), cubicPosition(1.0, 0.0, 2.0, 0.0, 2.0, 2.0), 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), cubicVelocity(1.0, 0.0, 2.0, 0.0, 2.0, 2.0), 1.0e-6);
}

test "standup goal matches original two-link IK bounds" {
    const goal = standupGoalJointPositions();
    try std.testing.expectEqual(@as(f32, 0.0), goal[0]);
    try std.testing.expect(goal[1] >= types.fl_joint_lower[1]);
    try std.testing.expect(goal[1] <= types.fl_joint_upper[1]);
    try std.testing.expect(goal[2] >= types.fl_joint_lower[2]);
    try std.testing.expect(goal[2] <= types.fl_joint_upper[2]);
}
