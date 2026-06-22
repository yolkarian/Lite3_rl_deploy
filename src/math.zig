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
    const a = (vf * duration - 2.0 * xf + v0 * duration + 2.0 * x0) / std.math.pow(f32, duration, 3.0);
    const b = (3.0 * xf - vf * duration - 2.0 * v0 * duration - 3.0 * x0) / std.math.pow(f32, duration, 2.0);
    return a * std.math.pow(f32, t, 3.0) + b * std.math.pow(f32, t, 2.0) + v0 * t + x0;
}

pub fn cubicVelocity(x0: f32, v0: f32, xf: f32, vf: f32, t: f32, duration: f32) f32 {
    if (duration <= 0.0 or t >= duration) return vf;
    if (t <= 0.0) return v0;
    const a = (vf * duration - 2.0 * xf + v0 * duration + 2.0 * x0) / std.math.pow(f32, duration, 3.0);
    const b = (3.0 * xf - vf * duration - 2.0 * v0 * duration - 3.0 * x0) / std.math.pow(f32, duration, 2.0);
    return 3.0 * a * std.math.pow(f32, t, 2.0) + 2.0 * b * t + v0;
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
