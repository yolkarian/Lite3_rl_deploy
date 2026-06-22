const std = @import("std");

pub const dof_count: usize = 12;
pub const action_dim: usize = 12;
pub const raw_obs_dim: usize = 117;
pub const obs_history_horizon: usize = 40;
pub const position_history_steps: usize = 3;
pub const velocity_history_steps: usize = 2;
pub const action_history_steps: usize = 2;

pub const gravity: f32 = 9.81;
pub const degrees_to_radians: f32 = std.math.pi / 180.0;
pub const radians_to_degrees: f32 = 180.0 / std.math.pi;

pub const Vec3 = [3]f32;
pub const JointVector = [dof_count]f32;
pub const RawObservation = [raw_obs_dim]f32;
pub const RawObservationHistory = [obs_history_horizon][raw_obs_dim]f32;

pub const joint_names = [_][]const u8{
    "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
    "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
    "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
    "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint",
};

/// Default joint order/values match `../legged-training/configs/env/lite3.yaml`.
pub const default_joint_positions: JointVector = .{
    0.0, -1.0, 1.8,
    0.0, -1.0, 1.8,
    0.0, -1.0, 1.8,
    0.0, -1.0, 1.8,
};

pub const joint_target_lower: JointVector = .{
    -0.523, -2.67, 0.524,
    -0.523, -2.67, 0.524,
    -0.523, -2.67, 0.524,
    -0.523, -2.67, 0.524,
};

pub const joint_target_upper: JointVector = .{
    0.523, 0.314, 2.792,
    0.523, 0.314, 2.792,
    0.523, 0.314, 2.792,
    0.523, 0.314, 2.792,
};

pub const MotionState = enum(i32) {
    waiting_for_stand = 0,
    standing_up = 1,
    joint_damping = 2,
    rl_control = 6,
};

pub const PolicyOutputKind = enum {
    /// Default from legged-training `export_policy_onnx.py --postprocess-output joint-target`.
    joint_target,
    /// ONNX output is a radian offset from `default_joint_positions`.
    action_offset,
    /// ONNX output is unclipped normalized policy action.
    policy_action,

    pub fn parse(text: []const u8) ?PolicyOutputKind {
        if (std.mem.eql(u8, text, "joint-target") or std.mem.eql(u8, text, "joint_target")) return .joint_target;
        if (std.mem.eql(u8, text, "action-offset") or std.mem.eql(u8, text, "action_offset")) return .action_offset;
        if (std.mem.eql(u8, text, "policy-action") or std.mem.eql(u8, text, "policy_action")) return .policy_action;
        return null;
    }
};

pub const UserCommand = struct {
    target_mode: MotionState = .waiting_for_stand,
    linear_x: f32 = 0.0,
    linear_y: f32 = 0.0,
    yaw_rate: f32 = 0.0,
};

pub const RobotState = struct {
    tick: u32 = 0,
    timestamp_s: f64 = 0.0,
    rpy_rad: Vec3 = .{ 0.0, 0.0, 0.0 },
    angular_velocity_rad_s: Vec3 = .{ 0.0, 0.0, 0.0 },
    linear_acc_m_s2: Vec3 = .{ 0.0, 0.0, gravity },
    joint_position: JointVector = std.mem.zeroes(JointVector),
    joint_velocity: JointVector = std.mem.zeroes(JointVector),
    joint_torque: JointVector = std.mem.zeroes(JointVector),
};

pub const JointCommand = struct {
    kp: f32 = 0.0,
    position: f32 = 0.0,
    kd: f32 = 0.0,
    velocity: f32 = 0.0,
    torque: f32 = 0.0,
};

pub const LowLevelCommand = [dof_count]JointCommand;

pub const Gains = struct {
    kp: f32,
    kd: f32,
};

pub const ControllerConfig = struct {
    policy_path: []const u8,
    robot_ip: []const u8 = "192.168.2.1",
    robot_port: u16 = 43893,
    output_kind: PolicyOutputKind = .joint_target,
    policy_decimation: u32 = 12,
    stand_duration_s: f64 = 2.0,
    damping_duration_s: f64 = 3.0,
    policy_action_scale: f32 = 0.25,
    clip_actions: f32 = 12.0,
    rl_gains: Gains = .{ .kp = 20.0, .kd = 0.7 },
    stand_gains: Gains = .{ .kp = 100.0, .kd = 2.5 },
    damping_gains: Gains = .{ .kp = 0.0, .kd = 2.5 },
    max_command: Vec3 = .{ 1.0, 1.0, 1.0 },
    auto_rl: bool = false,
    fixed_command: ?Vec3 = null,
    keyboard: bool = true,
    max_run_time_s: ?f64 = null,
};

pub fn zeroCommand() LowLevelCommand {
    return std.mem.zeroes(LowLevelCommand);
}

pub fn clamp(value: f32, lower: f32, upper: f32) f32 {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

pub fn clampJointTargets(targets: *JointVector) void {
    for (targets, 0..) |*target, index| {
        target.* = clamp(target.*, joint_target_lower[index], joint_target_upper[index]);
    }
}
