const std = @import("std");

const Platform = enum {
    x86_64,
    aarch64,
};

const TargetAndPlatform = struct {
    target: std.Build.ResolvedTarget,
    platform: Platform,
};

const RuntimeLibs = struct {
    motion_sdk: []const u8,
    motion_wrapped_sdk: []const u8,
    onnxruntime_dir: []const u8,
};

fn parsePlatform(platform_text: []const u8) Platform {
    if (std.mem.eql(u8, platform_text, "x86_64")) return .x86_64;
    if (std.mem.eql(u8, platform_text, "aarch64")) return .aarch64;
    std.debug.panic("unsupported platform '{s}', expected 'x86_64' or 'aarch64'", .{platform_text});
}

fn targetForPlatform(b: *std.Build, platform: Platform) std.Build.ResolvedTarget {
    const arch_os_abi = switch (platform) {
        .x86_64 => "x86_64-linux-gnu",
        .aarch64 => "aarch64-linux-gnu",
    };
    const query = std.Target.Query.parse(.{ .arch_os_abi = arch_os_abi }) catch unreachable;
    return b.resolveTargetQuery(query);
}

fn platformFromTarget(target: std.Build.ResolvedTarget) Platform {
    if (target.result.os.tag != .linux) {
        std.debug.panic("unsupported target OS '{s}', expected linux", .{@tagName(target.result.os.tag)});
    }
    if (!target.result.abi.isGnu()) {
        std.debug.panic("unsupported target ABI '{s}', expected gnu", .{@tagName(target.result.abi)});
    }

    return switch (target.result.cpu.arch) {
        .x86_64 => .x86_64,
        .aarch64 => .aarch64,
        else => |arch| std.debug.panic("unsupported target architecture '{s}', expected x86_64 or aarch64", .{@tagName(arch)}),
    };
}

fn resolveTargetAndPlatform(b: *std.Build) TargetAndPlatform {
    const standard_target = b.standardTargetOptions(.{});
    const platform_option = b.option([]const u8, "platform", "Legacy platform shortcut: x86_64 or aarch64");

    if (platform_option) |platform_text| {
        const platform = parsePlatform(platform_text);
        return .{
            .target = targetForPlatform(b, platform),
            .platform = platform,
        };
    }

    return .{
        .target = standard_target,
        .platform = platformFromTarget(standard_target),
    };
}

fn runtimeLibsForPlatform(platform: Platform) RuntimeLibs {
    return switch (platform) {
        .x86_64 => .{
            .motion_sdk = "libdeeprobotics_legged_sdk_x86_64.so",
            .motion_wrapped_sdk = "libdeeprobotics_legged_wrapped_sdk_x86_64.so",
            .onnxruntime_dir = "x86",
        },
        .aarch64 => .{
            .motion_sdk = "libdeeprobotics_legged_sdk_aarch64.so",
            .motion_wrapped_sdk = "libdeeprobotics_legged_wrapped_sdk_aarch64.so",
            .onnxruntime_dir = "arm",
        },
    };
}

fn onnxRuntimeLibPath(b: *std.Build, platform: Platform, basename: []const u8) std.Build.LazyPath {
    const libs = runtimeLibsForPlatform(platform);
    return b.path(b.fmt("third_party/onnxruntime/{s}/lib/{s}", .{ libs.onnxruntime_dir, basename }));
}

fn onnxRuntimeIncludePath(b: *std.Build, platform: Platform) std.Build.LazyPath {
    const libs = runtimeLibsForPlatform(platform);
    return b.path(b.fmt("third_party/onnxruntime/{s}/include", .{libs.onnxruntime_dir}));
}

fn addRuntimeLibCopies(b: *std.Build, platform: Platform) std.Build.LazyPath {
    const libs = runtimeLibsForPlatform(platform);
    const runtime_libs = b.addNamedWriteFiles("runtime_libs");

    const ort_versioned = b.path(b.fmt("third_party/onnxruntime/{s}/lib/libonnxruntime.so.1.22.0", .{libs.onnxruntime_dir}));
    _ = runtime_libs.addCopyFile(ort_versioned, "libonnxruntime.so.1.22.0");
    _ = runtime_libs.addCopyFile(ort_versioned, "libonnxruntime.so.1");
    _ = runtime_libs.addCopyFile(ort_versioned, "libonnxruntime.so");
    _ = runtime_libs.addCopyFile(
        b.path(b.fmt("third_party/onnxruntime/{s}/lib/libonnxruntime_providers_shared.so", .{libs.onnxruntime_dir})),
        "libonnxruntime_providers_shared.so",
    );

    _ = runtime_libs.addCopyFile(
        b.path(b.fmt("../Lite3_MotionSDK_Zig/lib/{s}", .{libs.motion_sdk})),
        libs.motion_sdk,
    );
    _ = runtime_libs.addCopyFile(
        b.path(b.fmt("../Lite3_MotionSDK_Zig/lib/{s}", .{libs.motion_wrapped_sdk})),
        libs.motion_wrapped_sdk,
    );

    const runtime_libs_dir = runtime_libs.getDirectory();
    b.addNamedLazyPath("runtime_libs", runtime_libs_dir);
    return runtime_libs_dir;
}

fn installRuntimeLibs(b: *std.Build, runtime_libs_dir: std.Build.LazyPath) void {
    b.installDirectory(.{
        .source_dir = runtime_libs_dir,
        .install_dir = .lib,
        .install_subdir = "",
    });
}

fn linkOnnxRuntime(module: *std.Build.Module, platform: Platform) void {
    const b = module.owner;
    module.addIncludePath(onnxRuntimeIncludePath(b, platform));
    module.addObjectFile(onnxRuntimeLibPath(b, platform, "libonnxruntime.so"));
    module.addRPathSpecial("$ORIGIN/../lib");
}

pub fn build(b: *std.Build) void {
    const target_and_platform = resolveTargetAndPlatform(b);
    const target = target_and_platform.target;
    const platform = target_and_platform.platform;
    const optimize = b.standardOptimizeOption(.{});

    const runtime_libs_dir = addRuntimeLibCopies(b, platform);
    installRuntimeLibs(b, runtime_libs_dir);

    const motion_dep = b.dependency("lite3_motion_sdk", .{
        .target = target,
        .optimize = optimize,
        .platform = @tagName(platform),
    });
    const motion_sdk_module = motion_dep.module("lite3_motion_sdk");

    const deploy_module = b.addModule("lite3_deploy", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .link_libcpp = true,
    });
    deploy_module.addImport("lite3_motion_sdk", motion_sdk_module);
    linkOnnxRuntime(deploy_module, platform);

    const lib = b.addLibrary(.{
        .name = "lite3_deploy",
        .root_module = deploy_module,
        .linkage = .static,
    });
    b.installArtifact(lib);

    const exe_module = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .link_libcpp = true,
    });
    exe_module.addImport("lite3_deploy", deploy_module);

    const exe = b.addExecutable(.{
        .name = "lite3-deploy",
        .root_module = exe_module,
    });
    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);

    const run_step = b.step("run", "Run Lite3 hardware deployment executable");
    run_step.dependOn(&run_cmd.step);

    const unit_tests = b.addTest(.{ .root_module = deploy_module });
    const test_cmd = b.addRunArtifact(unit_tests);
    test_cmd.step.dependOn(b.getInstallStep());

    const test_step = b.step("test", "Run Zig unit tests");
    test_step.dependOn(&test_cmd.step);
}
