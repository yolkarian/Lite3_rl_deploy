const std = @import("std");
const types = @import("types.zig");

pub const CommandSource = struct {
    command: types.UserCommand = .{},
    keyboard: bool,
    auto_rl: bool,
    fixed_command: ?types.Vec3,
    stdin_flags: ?c_int = null,

    pub fn init(config: types.ControllerConfig) CommandSource {
        var source = CommandSource{
            .keyboard = config.keyboard,
            .auto_rl = config.auto_rl,
            .fixed_command = config.fixed_command,
        };
        if (config.fixed_command) |cmd| {
            source.command.linear_x = cmd[0];
            source.command.linear_y = cmd[1];
            source.command.yaw_rate = cmd[2];
        }
        return source;
    }

    pub fn start(self: *CommandSource) void {
        if (!self.keyboard) return;
        const flags = std.c.fcntl(std.posix.STDIN_FILENO, std.c.F.GETFL, @as(c_int, 0));
        if (flags < 0) return;
        self.stdin_flags = flags;
        const nonblock_flag: c_int = @intCast(@as(u32, @bitCast(std.c.O{ .NONBLOCK = true })));
        _ = std.c.fcntl(std.posix.STDIN_FILENO, std.c.F.SETFL, @as(c_int, flags | nonblock_flag));
        std.debug.print(
            "Keyboard commands: z=stand, c=RL, r=damping, wasd=xy, q/e=yaw, x=zero velocity. Press Enter after keys.\n",
            .{},
        );
    }

    pub fn stop(self: *CommandSource) void {
        if (self.stdin_flags) |flags| {
            _ = std.c.fcntl(std.posix.STDIN_FILENO, std.c.F.SETFL, @as(c_int, flags));
            self.stdin_flags = null;
        }
    }

    pub fn poll(self: *CommandSource, feedback_state: types.MotionState) types.UserCommand {
        if (self.auto_rl) {
            switch (feedback_state) {
                .waiting_for_stand => self.command.target_mode = .standing_up,
                .standing_up => self.command.target_mode = .rl_control,
                else => {},
            }
        }

        if (self.fixed_command) |cmd| {
            self.command.linear_x = cmd[0];
            self.command.linear_y = cmd[1];
            self.command.yaw_rate = cmd[2];
        }

        if (self.keyboard) self.pollKeyboard(feedback_state);
        return self.command;
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
        switch (byte) {
            'z', 'Z' => {
                if (feedback_state == .waiting_for_stand) self.command.target_mode = .standing_up;
            },
            'c', 'C' => {
                if (feedback_state == .standing_up) self.command.target_mode = .rl_control;
            },
            'r', 'R' => self.command.target_mode = .joint_damping,
            'w', 'W' => self.command.linear_x = types.clamp(self.command.linear_x + 0.1, -1.0, 1.0),
            's', 'S' => self.command.linear_x = types.clamp(self.command.linear_x - 0.1, -1.0, 1.0),
            'a', 'A' => self.command.linear_y = types.clamp(self.command.linear_y + 0.1, -1.0, 1.0),
            'd', 'D' => self.command.linear_y = types.clamp(self.command.linear_y - 0.1, -1.0, 1.0),
            'q', 'Q' => self.command.yaw_rate = types.clamp(self.command.yaw_rate + 0.1, -1.0, 1.0),
            'e', 'E' => self.command.yaw_rate = types.clamp(self.command.yaw_rate - 0.1, -1.0, 1.0),
            'x', 'X', ' ' => {
                self.command.linear_x = 0.0;
                self.command.linear_y = 0.0;
                self.command.yaw_rate = 0.0;
            },
            else => {},
        }
    }
};
