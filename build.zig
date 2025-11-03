const std = @import("std");
const protobuf = @import("protobuf");
const zon = @import("build.zig.zon");
const Build = std.Build;

pub fn build(b: *Build) void {
    const host_target = b.standardTargetOptions(.{});
    const lw_mod = b.addModule("logicweave", .{ .root_source_file = b.path("src/root.zig") });

    // Add protobuf depedencies
    generate_protobuf_bindings(b, host_target);
    const protobuf_dep = b.dependency("protobuf", .{});

    // Fetch shared firmware options ONCE before the loop
    const commit_hash = b.option([]const u8, "GIT_HASH", "The git commit hash") orelse blk: {
        const stdout = b.run(&.{ "git", "rev-parse", "--short=8", "HEAD" });
        break :blk stdout[0 .. stdout.len - 1]; // remove the \n
    };
    const updated_at = b.run(&.{ "git", "log", "-1", "--format=%cd" });

    const lw_options = b.addOptions();

    lw_options.addOption([]const u8, "GIT_HASH", commit_hash);
    lw_options.addOption([]const u8, "UPDATED_AT", updated_at);
    lw_options.addOption([]const u8, "version", zon.version);

    lw_mod.addOptions("firmware_config", lw_options);

    // Add the imports into my logicweave module
    lw_mod.addImport("protobuf", protobuf_dep.module("protobuf"));
}

fn generate_protobuf_bindings(b: *std.Build, host_target: std.Build.ResolvedTarget) void {
    const protobuf_generated_path = "src/proto_gen/";
    const protobuf_def_path = "proto/";
    // Removed the single 'definition_file' declaration, as it's now handled by the loop variable.

    const definition_files: []const []const u8 = &.{ "logicweave.proto", "logicweave_core.proto" };

    // 1. Generate Python Protobuf code
    // We must ensure the step name is unique for each file (e.g., gen-python-proto-logicweave.proto)
    const py_step_name = "gen-python-proto";
    const gen_python_proto_cmd = b.addSystemCommand(&.{
        "protoc",
        "-I=" ++ protobuf_def_path,
        "--python_out=software/src/LogicWeave/proto_gen",
        protobuf_def_path ++ definition_files[0],
        protobuf_def_path ++ definition_files[1],
    });
    gen_python_proto_cmd.step.name = py_step_name;
    b.getInstallStep().dependOn(&gen_python_proto_cmd.step);

    // 2. Generate Zig Protobuf code
    const zig_step_name = "gen-zig-proto";
    const protobuf_dep = b.dependency("protobuf", .{ .target = host_target });
    const protoc_step = protobuf.RunProtocStep.create(protobuf_dep.builder, host_target, .{
        .destination_directory = b.path(protobuf_generated_path),
        .source_files = &.{ definition_files[0], definition_files[1] },
        .include_directories = &.{protobuf_def_path},
    });
    protoc_step.step.name = zig_step_name;

    // Make the Zig Protobuf step depend on the Python Protobuf step for this specific file

    b.getInstallStep().dependOn(&protoc_step.step);
}
