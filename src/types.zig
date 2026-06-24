const std = @import("std");

pub const dof_count: usize = 12;
pub const action_dim: usize = 12;
pub const raw_obs_dim: usize = 117;
pub const obs_history_horizon: usize = 40;
pub const position_history_steps: usize = 3;
pub const velocity_history_steps: usize = 2;
pub const action_history_steps: usize = 2;

pub const gravity: f32 = 9.815;
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

pub const body_len_x: f32 = 0.1745 * 2.0;
pub const body_len_y: f32 = 0.062 * 2.0;
pub const hip_len: f32 = 0.0985;
pub const thigh_len: f32 = 0.20;
pub const shank_len: f32 = 0.21;

pub const pre_height: f32 = 0.12;
pub const stand_height: f32 = 0.33;

/// Original C++ StandUpState final stand pose (ControlParameters::standup_default_joint_pos_).
pub const standup_default_joint_positions: JointVector = .{
    0.0, -0.6500, 1.3000,
    0.0, -0.6500, 1.3000,
    0.0, -0.6500, 1.3000,
    0.0, -0.6500, 1.3000,
};

/// Original ONNX runner policy default from legged-training lite3.yaml.
pub const policy_default_joint_positions: JointVector = .{
    0.0, -1.0, 1.8,
    0.0, -1.0, 1.8,
    0.0, -1.0, 1.8,
    0.0, -1.0, 1.8,
};

pub const policy_action_scales: JointVector = .{
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
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

pub const fl_joint_lower: Vec3 = .{ -0.530, -3.50, 0.349 };
pub const fl_joint_upper: Vec3 = .{ 0.530, 0.320, 2.80 };
pub const joint_velocity_limit: Vec3 = .{ 30.0, 30.0, 20.0 };
pub const torque_limit: Vec3 = .{ 40.0, 40.0, 65.0 };

pub const MotionState = enum(i32) {
    waiting_for_stand = 0,
    standing_up = 1,
    joint_damping = 2,
    rl_control = 6,
};

pub const CommandMode = enum {
    retroid,
    skydroid,
    keyboard,
    none,

    pub fn parse(text: []const u8) ?CommandMode {
        if (std.mem.eql(u8, text, "retroid")) return .retroid;
        if (std.mem.eql(u8, text, "skydroid")) return .skydroid;
        if (std.mem.eql(u8, text, "keyboard")) return .keyboard;
        if (std.mem.eql(u8, text, "none")) return .none;
        return null;
    }
};

pub const UserCommand = struct {
    soft_stop_flag: bool = false,
    target_mode: MotionState = .waiting_for_stand,
    target_gait: i32 = 0,
    forward_vel_scale: f32 = 0.0,
    side_vel_scale: f32 = 0.0,
    turnning_vel_scale: f32 = 0.0,
};

pub const RobotState = struct {
    tick: u32 = 0,
    timestamp_s: f64 = 0.0,
    rpy_rad: Vec3 = .{ 0.0, 0.0, 0.0 },
    /// Matches original HardwareInterface::GetImuOmega(): forwarded SDK
    /// angular_velocity_roll/pitch/yaw values without unit conversion.
    angular_velocity: Vec3 = .{ 0.0, 0.0, 0.0 },
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
    policy_path: []const u8 = "policy/ppo/policy.onnx",
    robot_ip: []const u8 = "192.168.2.1",
    robot_port: u16 = 43893,
    command_mode: CommandMode = .retroid,
    gamepad_port: u16 = 12121,
    policy_decimation: u32 = 12,
    stand_duration_s: f64 = 1.5,
    damping_duration_s: f64 = 3.0,
    clip_actions: f32 = 12.0,
    max_cmd_vel: Vec3 = .{ 0.8, 0.8, 0.8 },
    rl_gains: Gains = .{ .kp = 17.0, .kd = 0.9 },
    swing_leg_gains: Gains = .{ .kp = 100.0, .kd = 2.5 },
    damping_gains: Gains = .{ .kp = 0.0, .kd = 2.5 },
    auto_rl: bool = false,
    fixed_command: ?Vec3 = null,
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

pub fn jointOffsetFromPolicyDefault(position: JointVector) JointVector {
    var offset: JointVector = undefined;
    for (0..dof_count) |i| offset[i] = position[i] - policy_default_joint_positions[i];
    return offset;
}
