const std = @import("std");
const types = @import("types.zig");

const c = @cImport({
    @cInclude("onnxruntime_c_api.h");
});

pub const PolicyError = error{
    OnnxRuntime,
    InvalidOutputShape,
    InvalidOutputSize,
};

const input_names = [_][*:0]const u8{ "raw_obs", "raw_obs_history" };
const output_names = [_][*:0]const u8{"policy_action"};

pub const PolicySession = struct {
    allocator: std.mem.Allocator,
    api: [*c]const c.OrtApi,
    env: ?*c.OrtEnv = null,
    session_options: ?*c.OrtSessionOptions = null,
    session: ?*c.OrtSession = null,
    memory_info: ?*c.OrtMemoryInfo = null,
    model_path_z: [:0]u8,
    clip_actions: f32,

    pub fn init(allocator: std.mem.Allocator, model_path: []const u8, clip_actions: f32) !PolicySession {
        const model_path_z = try allocator.dupeZ(u8, model_path);
        std.debug.print("[ONNX INIT] Loading model: {s}\n", .{model_path});

        const base = c.OrtGetApiBase();
        const api = base.*.GetApi.?(c.ORT_API_VERSION);

        var self = PolicySession{
            .allocator = allocator,
            .api = api,
            .model_path_z = model_path_z,
            .clip_actions = clip_actions,
        };
        errdefer self.deinit();

        try self.check(api.*.CreateEnv.?(c.ORT_LOGGING_LEVEL_WARNING, "ONNXPolicy", &self.env));
        try self.check(api.*.CreateSessionOptions.?(&self.session_options));
        try self.check(api.*.SetIntraOpNumThreads.?(self.session_options, 1));
        try self.check(api.*.SetSessionGraphOptimizationLevel.?(self.session_options, c.ORT_ENABLE_ALL));
        try self.check(api.*.CreateCpuMemoryInfo.?(c.OrtArenaAllocator, c.OrtMemTypeDefault, &self.memory_info));
        try self.check(api.*.CreateSession.?(self.env, self.model_path_z.ptr, self.session_options, &self.session));
        std.debug.print("[ONNX INIT] Model loaded successfully.\n", .{});
        self.displayPolicyInfo();

        try self.warmup();
        return self;
    }

    pub fn deinit(self: *PolicySession) void {
        if (self.session) |session| self.api.*.ReleaseSession.?(session);
        if (self.session_options) |session_options| self.api.*.ReleaseSessionOptions.?(session_options);
        if (self.memory_info) |memory_info| self.api.*.ReleaseMemoryInfo.?(memory_info);
        if (self.env) |env| self.api.*.ReleaseEnv.?(env);
        if (self.model_path_z.len > 0) self.allocator.free(self.model_path_z);
        self.* = undefined;
    }

    pub fn run(
        self: *PolicySession,
        raw_obs: *types.RawObservation,
        raw_obs_history: *types.RawObservationHistory,
    ) !types.JointVector {
        const policy_action = try self.runPolicyAction(raw_obs, raw_obs_history);
        return self.postprocessPolicyAction(policy_action);
    }

    fn warmup(self: *PolicySession) !void {
        var raw_obs = std.mem.zeroes(types.RawObservation);
        var raw_history = std.mem.zeroes(types.RawObservationHistory);
        _ = try self.run(&raw_obs, &raw_history);
        std.debug.print("test_onnx ONNX policy network test success\n", .{});
        _ = try self.run(&raw_obs, &raw_history);
        std.debug.print("test_onnx ONNX policy network test success\n", .{});
    }

    fn displayPolicyInfo(self: *PolicySession) void {
        std.debug.print("ONNX policy: test_onnx\n", .{});
        std.debug.print("path: {s}\n", .{self.model_path_z});
        std.debug.print("raw_obs_dim: {}, obs_history: {}x{}, action_dim: {}\n", .{ types.raw_obs_dim, types.obs_history_horizon, types.raw_obs_dim, types.action_dim });
        std.debug.print("ONNX output: raw policy_action; Zig applies action_scale + default_pos\n", .{});
    }

    fn runPolicyAction(
        self: *PolicySession,
        raw_obs: *types.RawObservation,
        raw_obs_history: *types.RawObservationHistory,
    ) !types.JointVector {
        var obs_value: ?*c.OrtValue = null;
        var history_value: ?*c.OrtValue = null;
        var output_value: ?*c.OrtValue = null;
        defer if (obs_value) |value| self.api.*.ReleaseValue.?(value);
        defer if (history_value) |value| self.api.*.ReleaseValue.?(value);
        defer if (output_value) |value| self.api.*.ReleaseValue.?(value);

        var obs_shape = [_]i64{@as(i64, @intCast(types.raw_obs_dim))};
        var history_shape = [_]i64{ @as(i64, @intCast(types.obs_history_horizon)), @as(i64, @intCast(types.raw_obs_dim)) };

        try self.check(self.api.*.CreateTensorWithDataAsOrtValue.?(
            self.memory_info,
            @ptrCast(raw_obs.ptr),
            types.raw_obs_dim * @sizeOf(f32),
            &obs_shape,
            obs_shape.len,
            c.ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
            &obs_value,
        ));
        try self.check(self.api.*.CreateTensorWithDataAsOrtValue.?(
            self.memory_info,
            @ptrCast(raw_obs_history.ptr),
            types.obs_history_horizon * types.raw_obs_dim * @sizeOf(f32),
            &history_shape,
            history_shape.len,
            c.ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
            &history_value,
        ));

        var ort_inputs = [_]?*c.OrtValue{ obs_value, history_value };
        var ort_outputs = [_]?*c.OrtValue{null};
        try self.check(self.api.*.Run.?(
            self.session,
            null,
            @ptrCast(&input_names),
            @ptrCast(&ort_inputs),
            ort_inputs.len,
            @ptrCast(&output_names),
            output_names.len,
            @ptrCast(&ort_outputs),
        ));
        output_value = ort_outputs[0];

        var data_ptr: ?*anyopaque = null;
        try self.check(self.api.*.GetTensorMutableData.?(output_value, &data_ptr));
        const action_ptr: [*]f32 = @ptrCast(@alignCast(data_ptr orelse return PolicyError.InvalidOutputSize));

        const output_len = try self.outputElementCount(output_value);
        if (output_len < types.action_dim) return PolicyError.InvalidOutputSize;

        var action: types.JointVector = undefined;
        for (0..types.action_dim) |index| action[index] = action_ptr[index];
        return action;
    }

    fn postprocessPolicyAction(self: *PolicySession, policy_action: types.JointVector) types.JointVector {
        var target: types.JointVector = undefined;
        for (0..types.action_dim) |index| {
            const clipped_action = types.clamp(policy_action[index], -self.clip_actions, self.clip_actions);
            target[index] = types.policy_default_joint_positions[index] + types.policy_action_scales[index] * clipped_action;
        }
        types.clampJointTargets(&target);
        return target;
    }

    fn outputElementCount(self: *PolicySession, output_value: ?*c.OrtValue) !usize {
        var shape_info: ?*c.OrtTensorTypeAndShapeInfo = null;
        defer if (shape_info) |info| self.api.*.ReleaseTensorTypeAndShapeInfo.?(info);
        try self.check(self.api.*.GetTensorTypeAndShape.?(output_value, &shape_info));

        var dim_count: usize = 0;
        try self.check(self.api.*.GetDimensionsCount.?(shape_info, &dim_count));
        if (dim_count == 0 or dim_count > 4) return PolicyError.InvalidOutputShape;

        var dims = [_]i64{ 1, 1, 1, 1 };
        try self.check(self.api.*.GetDimensions.?(shape_info, &dims, dim_count));

        var count: usize = 1;
        for (dims[0..dim_count]) |dim| {
            if (dim <= 0) return PolicyError.InvalidOutputShape;
            count *= @intCast(dim);
        }
        return count;
    }

    fn check(self: *PolicySession, status: ?*c.OrtStatus) PolicyError!void {
        if (status == null) return;
        const message = self.api.*.GetErrorMessage.?(status);
        std.debug.print("ONNX Runtime error: {s}\n", .{message});
        self.api.*.ReleaseStatus.?(status.?);
        return PolicyError.OnnxRuntime;
    }
};

test "original ONNX policy_action output runs and postprocesses to joint targets" {
    var session = try PolicySession.init(std.testing.allocator, "policy/ppo/policy.onnx", 12.0);
    defer session.deinit();

    var raw_obs = std.mem.zeroes(types.RawObservation);
    var raw_obs_history = std.mem.zeroes(types.RawObservationHistory);
    const target = try session.run(&raw_obs, &raw_obs_history);

    for (target, 0..) |value, index| {
        try std.testing.expect(std.math.isFinite(value));
        try std.testing.expect(value >= types.joint_target_lower[index] - 1.0e-5);
        try std.testing.expect(value <= types.joint_target_upper[index] + 1.0e-5);
    }
}
