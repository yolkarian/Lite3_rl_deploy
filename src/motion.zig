const std = @import("std");
const sdk = @import("lite3_motion_sdk");
const types = @import("types.zig");

const MotionError = error{
    ReceiverCreateFailed,
    SenderCreateFailed,
    StateUnavailable,
};

fn onMessageUpdate(code: c_int, ptr: ?*anyopaque) callconv(.c) void {
    if (code != 0x0906) return;
    const raw = ptr orelse return;
    const updated: *bool = @ptrCast(@alignCast(raw));
    updated.* = true;
}

pub const HardwareInterface = struct {
    allocator: std.mem.Allocator,
    robot_ip_z: [:0]u8,
    sender: sdk.SenderHandle,
    receiver: *sdk.ReceiverHandle,
    data: ?[*c]sdk.RobotData = null,
    command: sdk.RobotCmd = std.mem.zeroes(sdk.RobotCmd),
    message_updated: bool = false,
    started: bool = false,

    pub fn init(allocator: std.mem.Allocator, robot_ip: []const u8, robot_port: u16) !HardwareInterface {
        const ip_z = try allocator.dupeZ(u8, robot_ip);
        errdefer allocator.free(ip_z);

        const receiver = sdk.Receiver_create() orelse return MotionError.ReceiverCreateFailed;
        errdefer sdk.Receiver_destroy(receiver);

        const sender = sdk.Sender_createWithIpPort(ip_z.ptr, robot_port) orelse return MotionError.SenderCreateFailed;
        errdefer sdk.Sender_destroy(sender);

        sdk.Receiver_registerCallback(receiver, onMessageUpdate, null);
        sdk.Sender_robotStateInit(sender);

        return .{
            .allocator = allocator,
            .robot_ip_z = ip_z,
            .sender = sender,
            .receiver = receiver,
        };
    }

    pub fn deinit(self: *HardwareInterface) void {
        if (self.started) self.stop();
        sdk.Sender_destroy(self.sender);
        sdk.Receiver_destroy(self.receiver);
        self.allocator.free(self.robot_ip_z);
    }

    pub fn start(self: *HardwareInterface) void {
        if (self.started) return;
        sdk.Receiver_registerCallback(self.receiver, onMessageUpdate, &self.message_updated);
        self.data = sdk.Receiver_getState(self.receiver);
        sdk.Receiver_startWork(self.receiver);
        sdk.Sender_robotStateInit(self.sender);
        sdk.Sender_controlGet(self.sender, sdk.SDK);
        self.started = true;
    }

    pub fn stop(self: *HardwareInterface) void {
        if (!self.started) return;
        self.send(types.zeroCommand()) catch {};
        sdk.Sender_controlGet(self.sender, sdk.ROBOT);
        self.started = false;
    }

    pub fn readState(self: *HardwareInterface) !types.RobotState {
        const data_ptr = self.data orelse return MotionError.StateUnavailable;
        const data = data_ptr[0];
        var state = types.RobotState{
            .tick = data.tick,
            .timestamp_s = @as(f64, @floatFromInt(data.tick)) * 0.001,
        };

        const imu = data.imu.unnamed_0.unnamed_0;
        state.rpy_rad = .{
            imu.angle_roll * types.degrees_to_radians,
            imu.angle_pitch * types.degrees_to_radians,
            imu.angle_yaw * types.degrees_to_radians,
        };
        state.angular_velocity_rad_s = .{
            imu.angular_velocity_roll * types.degrees_to_radians,
            imu.angular_velocity_pitch * types.degrees_to_radians,
            imu.angular_velocity_yaw * types.degrees_to_radians,
        };
        state.linear_acc_m_s2 = .{ imu.acc_x, imu.acc_y, imu.acc_z };

        for (0..types.dof_count) |index| {
            const joint = data.joint_data.unnamed_0.joint_data[index];
            state.joint_position[index] = joint.position;
            state.joint_velocity[index] = joint.velocity;
            state.joint_torque[index] = joint.torque;
        }
        return state;
    }

    pub fn send(self: *HardwareInterface, command: types.LowLevelCommand) !void {
        for (command, 0..) |joint_command, index| {
            self.command.unnamed_0.joint_cmd[index].kp = joint_command.kp;
            self.command.unnamed_0.joint_cmd[index].position = joint_command.position;
            self.command.unnamed_0.joint_cmd[index].kd = joint_command.kd;
            self.command.unnamed_0.joint_cmd[index].velocity = joint_command.velocity;
            self.command.unnamed_0.joint_cmd[index].torque = joint_command.torque;
        }
        sdk.Sender_sendCmd(self.sender, &self.command);
    }
};
