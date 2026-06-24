const std = @import("std");
const types = @import("types.zig");

const header = [_]u8{ 0x55, 0x66 };
const channel_count: usize = 16;
const joystick_range: f32 = 1000.0;
const retroid_id: u8 = 1;
const skydroid_id: u8 = 2;
const packet_header_len: usize = 10;
const packet_data_len: usize = channel_count * @sizeOf(u16);
const min_packet_len: usize = packet_header_len + packet_data_len;

pub const CommandSource = struct {
    mode: types.CommandMode,
    gamepad_port: u16,
    command: types.UserCommand = .{},
    last_feedback_state: types.MotionState = .waiting_for_stand,
    auto_rl: bool,
    fixed_command: ?types.Vec3,
    udp_fd: ?c_int = null,
    stdin_flags: ?c_int = null,
    stdin_termios: ?std.c.termios = null,
    retroid_last: RetroidKeys = .{},
    retroid_have_last: bool = false,
    skydroid_last: SkydroidKeys = .{},
    skydroid_have_last: bool = false,

    pub fn init(config: types.ControllerConfig) CommandSource {
        var source = CommandSource{
            .mode = config.command_mode,
            .gamepad_port = config.gamepad_port,
            .auto_rl = config.auto_rl,
            .fixed_command = config.fixed_command,
        };
        if (config.fixed_command) |cmd| {
            source.command.forward_vel_scale = cmd[0];
            source.command.side_vel_scale = cmd[1];
            source.command.turnning_vel_scale = cmd[2];
        }
        return source;
    }

    pub fn start(self: *CommandSource) void {
        switch (self.mode) {
            .retroid => self.startUdp("Retroid"),
            .skydroid => self.startUdp("Skydroid"),
            .keyboard => self.startKeyboard(),
            .none => std.debug.print("Using no user command interface\n", .{}),
        }
    }

    pub fn stop(self: *CommandSource) void {
        if (self.udp_fd) |fd| {
            _ = std.posix.system.close(fd);
            self.udp_fd = null;
        }
        if (self.stdin_termios) |termios| {
            _ = std.c.tcsetattr(std.posix.STDIN_FILENO, std.c.TCSA.NOW, &termios);
            self.stdin_termios = null;
        }
        if (self.stdin_flags) |flags| {
            _ = std.c.fcntl(std.posix.STDIN_FILENO, std.c.F.SETFL, @as(c_int, flags));
            self.stdin_flags = null;
        }
    }

    pub fn poll(self: *CommandSource, feedback_state: types.MotionState) types.UserCommand {
        if (feedback_state != self.last_feedback_state) {
            self.last_feedback_state = feedback_state;
            self.command.target_mode = feedback_state;
        }

        if (self.auto_rl) {
            switch (feedback_state) {
                .waiting_for_stand => self.command.target_mode = .standing_up,
                .standing_up => self.command.target_mode = .rl_control,
                else => {},
            }
        }

        switch (self.mode) {
            .retroid => self.pollRetroid(feedback_state),
            .skydroid => self.pollSkydroid(feedback_state),
            .keyboard => self.pollKeyboard(feedback_state),
            .none => {},
        }

        if (self.fixed_command) |cmd| {
            self.command.forward_vel_scale = cmd[0];
            self.command.side_vel_scale = cmd[1];
            self.command.turnning_vel_scale = cmd[2];
        }

        return self.command;
    }

    fn startUdp(self: *CommandSource, name: []const u8) void {
        std.debug.print("Using {s} Gamepad Command Interface on UDP port {}\n", .{ name, self.gamepad_port });
        const fd = std.c.socket(std.posix.AF.INET, std.posix.SOCK.DGRAM, 0);
        if (fd < 0) {
            std.debug.print("failed to create gamepad UDP socket\n", .{});
            return;
        }
        var reuse: c_int = 1;
        _ = std.c.setsockopt(fd, std.posix.SOL.SOCKET, std.posix.SO.REUSEADDR, &reuse, @sizeOf(c_int));

        var addr = std.posix.sockaddr.in{
            .port = std.mem.nativeToBig(u16, self.gamepad_port),
            .addr = 0,
        };
        if (std.c.bind(fd, @ptrCast(&addr), @sizeOf(std.posix.sockaddr.in)) != 0) {
            std.debug.print("failed to bind gamepad UDP socket on port {}\n", .{self.gamepad_port});
            _ = std.posix.system.close(fd);
            return;
        }

        const flags = std.c.fcntl(fd, std.c.F.GETFL, @as(c_int, 0));
        if (flags >= 0) {
            const nonblock_flag: c_int = @intCast(@as(u32, @bitCast(std.c.O{ .NONBLOCK = true })));
            _ = std.c.fcntl(fd, std.c.F.SETFL, @as(c_int, flags | nonblock_flag));
        }
        self.udp_fd = fd;
    }

    fn startKeyboard(self: *CommandSource) void {
        std.debug.print("Using Keyboard Command Interface (stdin)\n", .{});
        const flags = std.c.fcntl(std.posix.STDIN_FILENO, std.c.F.GETFL, @as(c_int, 0));
        if (flags < 0) {
            std.debug.print("  stdin is unavailable; keyboard commands disabled.\n", .{});
            return;
        }
        self.stdin_flags = flags;

        var raw_mode_enabled = false;
        var old_termios: std.c.termios = undefined;
        if (std.c.tcgetattr(std.posix.STDIN_FILENO, &old_termios) == 0) {
            self.stdin_termios = old_termios;
            var raw = old_termios;
            raw.lflag.ICANON = false;
            raw.lflag.ECHO = false;
            raw_mode_enabled = std.c.tcsetattr(std.posix.STDIN_FILENO, std.c.TCSA.NOW, &raw) == 0;
        }

        const nonblock_flag: c_int = @intCast(@as(u32, @bitCast(std.c.O{ .NONBLOCK = true })));
        _ = std.c.fcntl(std.posix.STDIN_FILENO, std.c.F.SETFL, @as(c_int, flags | nonblock_flag));
        printKeyboardPrompt(raw_mode_enabled);
    }

    fn printKeyboardPrompt(raw_mode_enabled: bool) void {
        std.debug.print(
            \\  stdin mode: {s}
            \\  state keys: z = stand, c = enter RL, r = damping
            \\  RL command: w/s = vx +/-0.1, a/d = vy +/-0.1, q/e = yaw +/-0.1
            \\              x or Space = zero velocity
            \\
        , .{if (raw_mode_enabled) "raw nonblocking; press keys directly (no Enter)" else "nonblocking; raw TTY mode unavailable"});
    }

    fn pollRetroid(self: *CommandSource, feedback_state: types.MotionState) void {
        var buf: [1024]u8 = undefined;
        while (self.recvPacket(&buf)) |packet| {
            const keys = parseRetroid(packet) orelse continue;
            if (!self.retroid_have_last) {
                self.retroid_last = keys;
                self.retroid_have_last = true;
                continue;
            }

            self.command.forward_vel_scale = clampAxis(keys.left_axis_y);
            self.command.side_vel_scale = clampAxis(-keys.left_axis_x);
            self.command.turnning_vel_scale = clampAxis(-keys.right_axis_x);

            if (keys.value != self.retroid_last.value or !sameAxes(keys.axis_values, self.retroid_last.axis_values)) {
                switch (feedback_state) {
                    .waiting_for_stand => {
                        if (keys.bit(.Y) != self.retroid_last.bit(.Y)) self.command.target_mode = .standing_up;
                    },
                    .standing_up => {
                        if (keys.bit(.A) != self.retroid_last.bit(.A)) self.command.target_mode = .rl_control;
                    },
                    else => {},
                }
                if (keys.bit(.left_axis_button) and keys.bit(.right_axis_button)) self.command.target_mode = .joint_damping;
                self.retroid_last = keys;
            }
        }
    }

    fn pollSkydroid(self: *CommandSource, feedback_state: types.MotionState) void {
        var buf: [1024]u8 = undefined;
        while (self.recvPacket(&buf)) |packet| {
            const keys = parseSkydroid(packet) orelse continue;
            if (!self.skydroid_have_last) {
                self.skydroid_last = keys;
                self.skydroid_have_last = true;
                continue;
            }

            self.command.forward_vel_scale = clampAxis(keys.left_axis_y);
            self.command.side_vel_scale = clampAxis(-keys.left_axis_x);
            self.command.turnning_vel_scale = clampAxis(-keys.right_axis_x);

            if (keys.keys_value != self.skydroid_last.keys_value or !sameAxes(keys.axis_values, self.skydroid_last.axis_values) or !sameSwitches(keys.switch_keys, self.skydroid_last.switch_keys)) {
                switch (feedback_state) {
                    .waiting_for_stand => {
                        if (keys.bit(.A) != self.skydroid_last.bit(.A)) self.command.target_mode = .standing_up;
                    },
                    .standing_up => {
                        if (keys.bit(.right) != self.skydroid_last.bit(.right)) self.command.target_mode = .rl_control;
                    },
                    else => {},
                }
                if (keys.switch_keys[0] == 2 and keys.switch_keys[3] == 2) self.command.target_mode = .joint_damping;
                self.skydroid_last = keys;
            }
        }
    }

    fn recvPacket(self: *CommandSource, buf: *[1024]u8) ?[]const u8 {
        const fd = self.udp_fd orelse return null;
        const n = std.c.recv(fd, buf, buf.len, 0);
        if (n <= 0) return null;
        return buf[0..@intCast(n)];
    }

    fn pollKeyboard(self: *CommandSource, feedback_state: types.MotionState) void {
        var buf: [64]u8 = undefined;
        while (true) {
            const n = std.posix.read(std.posix.STDIN_FILENO, &buf) catch |err| switch (err) {
                error.WouldBlock => return,
                error.InputOutput => return,
                else => return,
            };
            if (n == 0) return;
            for (buf[0..n]) |byte| self.handleKey(byte, feedback_state);
            if (n < buf.len) return;
        }
    }

    fn handleKey(self: *CommandSource, byte: u8, feedback_state: types.MotionState) void {
        if (byte == 'r' or byte == 'R') {
            self.command.target_mode = .joint_damping;
            return;
        }

        switch (feedback_state) {
            .waiting_for_stand => switch (byte) {
                'z', 'Z' => self.command.target_mode = .standing_up,
                else => {},
            },
            .standing_up => switch (byte) {
                'c', 'C' => self.command.target_mode = .rl_control,
                else => {},
            },
            .rl_control => switch (byte) {
                'w', 'W' => self.command.forward_vel_scale = types.clamp(self.command.forward_vel_scale + 0.1, -1.0, 1.0),
                's', 'S' => self.command.forward_vel_scale = types.clamp(self.command.forward_vel_scale - 0.1, -1.0, 1.0),
                'a', 'A' => self.command.side_vel_scale = types.clamp(self.command.side_vel_scale + 0.1, -1.0, 1.0),
                'd', 'D' => self.command.side_vel_scale = types.clamp(self.command.side_vel_scale - 0.1, -1.0, 1.0),
                'q', 'Q' => self.command.turnning_vel_scale = types.clamp(self.command.turnning_vel_scale + 0.1, -1.0, 1.0),
                'e', 'E' => self.command.turnning_vel_scale = types.clamp(self.command.turnning_vel_scale - 0.1, -1.0, 1.0),
                'x', 'X', ' ' => self.zeroVelocity(),
                else => {},
            },
            else => {},
        }
    }

    fn zeroVelocity(self: *CommandSource) void {
        self.command.forward_vel_scale = 0.0;
        self.command.side_vel_scale = 0.0;
        self.command.turnning_vel_scale = 0.0;
    }
};

const RetroidButton = enum(u4) {
    R1 = 0,
    L1 = 1,
    start = 2,
    select = 3,
    R2 = 4,
    L2 = 5,
    A = 6,
    B = 7,
    X = 8,
    Y = 9,
    left = 10,
    right = 11,
    up = 12,
    down = 13,
    left_axis_button = 14,
    right_axis_button = 15,
};

const RetroidKeys = struct {
    value: u16 = 0,
    axis_values: [4]f32 = .{ 0.0, 0.0, 0.0, 0.0 },
    left_axis_x: f32 = 0.0,
    left_axis_y: f32 = 0.0,
    right_axis_x: f32 = 0.0,
    right_axis_y: f32 = 0.0,

    fn bit(self: RetroidKeys, button: RetroidButton) bool {
        return (self.value & (@as(u16, 1) << @intFromEnum(button))) != 0;
    }
};

const SkydroidButton = enum(u3) {
    C = 0,
    right = 1,
    D = 2,
    E = 3,
    F = 4,
    reserved = 5,
    A = 6,
    B = 7,
};

const SkydroidKeys = struct {
    keys_value: u8 = 0,
    switch_keys: [4]u8 = .{ 0, 0, 0, 0 },
    axis_values: [4]f32 = .{ 0.0, 0.0, 0.0, 0.0 },
    left_axis_x: f32 = 0.0,
    left_axis_y: f32 = 0.0,
    right_axis_x: f32 = 0.0,
    right_axis_y: f32 = 0.0,

    fn bit(self: SkydroidKeys, button: SkydroidButton) bool {
        return (self.keys_value & (@as(u8, 1) << @intFromEnum(button))) != 0;
    }
};

fn parseRetroid(packet: []const u8) ?RetroidKeys {
    if (!packetValid(packet, retroid_id)) return null;
    const data = packet[packet_header_len .. packet_header_len + packet_data_len];
    var keys = RetroidKeys{};

    var value: u16 = 0;
    for (0..channel_count) |i| {
        const raw = readI16(data, i * 2);
        if (raw != 0) value |= @as(u16, 1) << @intCast(i);
    }

    const left_axis_x = readI16(data, 20);
    const left_axis_y = readI16(data, 22);
    const right_axis_x = readI16(data, 24);
    const right_axis_y = readI16(data, 26);

    value = setBit(value, @intFromEnum(RetroidButton.left), left_axis_x == -1000);
    value = setBit(value, @intFromEnum(RetroidButton.right), left_axis_x == 1000);
    value = setBit(value, @intFromEnum(RetroidButton.up), left_axis_y == 1000);
    value = setBit(value, @intFromEnum(RetroidButton.down), left_axis_y == -1000);

    keys.value = value;
    keys.left_axis_x = @as(f32, @floatFromInt(left_axis_x)) / joystick_range;
    keys.left_axis_y = @as(f32, @floatFromInt(left_axis_y)) / joystick_range;
    keys.right_axis_x = @as(f32, @floatFromInt(right_axis_x)) / joystick_range;
    keys.right_axis_y = @as(f32, @floatFromInt(right_axis_y)) / joystick_range;
    keys.axis_values = .{ keys.left_axis_x, keys.left_axis_y, keys.right_axis_x, keys.right_axis_y };
    return keys;
}

fn parseSkydroid(packet: []const u8) ?SkydroidKeys {
    if (!packetValid(packet, skydroid_id)) return null;
    const data = packet[packet_header_len .. packet_header_len + packet_data_len];
    var keys = SkydroidKeys{};

    var value: u8 = 0;
    for (0..8) |i| {
        const raw = readI16(data, 16 + i * 2);
        if (raw != 0) value |= @as(u8, 1) << @intCast(i);
    }

    const right_axis_x = readI16(data, 0);
    const right_axis_y = readI16(data, 2);
    const left_axis_y = readI16(data, 4);
    const left_axis_x = readI16(data, 6);

    keys.keys_value = value;
    keys.left_axis_x = @as(f32, @floatFromInt(left_axis_x)) / joystick_range;
    keys.left_axis_y = @as(f32, @floatFromInt(left_axis_y)) / joystick_range;
    keys.right_axis_x = @as(f32, @floatFromInt(right_axis_x)) / joystick_range;
    keys.right_axis_y = @as(f32, @floatFromInt(right_axis_y)) / joystick_range;
    keys.axis_values = .{ keys.left_axis_x, keys.left_axis_y, keys.right_axis_x, keys.right_axis_y };
    for (0..4) |i| keys.switch_keys[i] = @intCast(readU16(data, 8 + i * 2));
    return keys;
}

fn packetValid(packet: []const u8, expected_id: u8) bool {
    if (packet.len < min_packet_len) return false;
    if (packet[0] != header[0] or packet[1] != header[1]) return false;
    if (packet[7] != expected_id) return false;

    const data_len = readU16(packet, 3);
    if (data_len > packet_data_len) return false;
    if (packet.len < packet_header_len + data_len) return false;

    const crc = readU16(packet, 8);
    var sum: u16 = 0;
    for (packet[packet_header_len .. packet_header_len + data_len]) |byte| sum +%= byte;
    return crc == sum;
}

fn readU16(data: []const u8, offset: usize) u16 {
    return std.mem.readInt(u16, data[offset..][0..2], .little);
}

fn readI16(data: []const u8, offset: usize) i16 {
    return std.mem.readInt(i16, data[offset..][0..2], .little);
}

fn setBit(value: u16, bit_index: u4, enabled: bool) u16 {
    const mask = @as(u16, 1) << bit_index;
    return if (enabled) (value | mask) else (value & ~mask);
}

fn sameAxes(a: [4]f32, b: [4]f32) bool {
    for (0..4) |i| if (a[i] != b[i]) return false;
    return true;
}

fn sameSwitches(a: [4]u8, b: [4]u8) bool {
    for (0..4) |i| if (a[i] != b[i]) return false;
    return true;
}

fn clampAxis(value: f32) f32 {
    return types.clamp(value, -1.0, 1.0);
}

test "retroid packet parser maps original channel order" {
    var packet = [_]u8{0} ** min_packet_len;
    packet[0] = 0x55;
    packet[1] = 0x66;
    std.mem.writeInt(u16, packet[3..5], packet_data_len, .little);
    packet[7] = retroid_id;

    const data = packet[packet_header_len .. packet_header_len + packet_data_len];
    std.mem.writeInt(i16, data[12..14], 1, .little); // A button channel 6
    std.mem.writeInt(i16, data[18..20], 1, .little); // Y button channel 9
    std.mem.writeInt(i16, data[20..22], -1000, .little); // left_axis_x
    std.mem.writeInt(i16, data[22..24], 500, .little); // left_axis_y
    std.mem.writeInt(i16, data[24..26], -250, .little); // right_axis_x
    std.mem.writeInt(i16, data[28..30], 1, .little); // left axis button
    std.mem.writeInt(i16, data[30..32], 1, .little); // right axis button

    var crc: u16 = 0;
    for (data) |byte| crc +%= byte;
    std.mem.writeInt(u16, packet[8..10], crc, .little);

    const keys = parseRetroid(&packet).?;
    try std.testing.expect(keys.bit(.A));
    try std.testing.expect(keys.bit(.Y));
    try std.testing.expect(keys.bit(.left));
    try std.testing.expect(keys.bit(.left_axis_button));
    try std.testing.expect(keys.bit(.right_axis_button));
    try std.testing.expectEqual(@as(f32, -1.0), keys.left_axis_x);
    try std.testing.expectEqual(@as(f32, 0.5), keys.left_axis_y);
    try std.testing.expectEqual(@as(f32, -0.25), keys.right_axis_x);
}
