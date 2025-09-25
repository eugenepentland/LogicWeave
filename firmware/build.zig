const std = @import("std");
const microzig = @import("microzig");
const protobuf = @import("protobuf");
const Build = std.Build;
const Module = Build.Module;

// Use the definition provided by the user
const Target = microzig.Target;
const HardwareAbstractionLayer = microzig.HardwareAbstractionLayer;

const Example = struct {
    target: *const Target,
    name: []const u8,
    file: []const u8,
    generic: bool,
};

// --- Function to handle all Protobuf operations ---
fn add_protobuf_handling(b: *Build, optimize: std.builtin.OptimizeMode, host_target: std.Build.ResolvedTarget) *Module {
    const protobuf_generated_path = "src/proto_gen/";
    const protobuf_def_path = "../proto/";
    const definition_file = "all.proto";

    // 1. Generate Python Protobuf code
    const gen_python_proto_cmd = b.addSystemCommand(&.{
        "protoc",
        "-I=" ++ protobuf_def_path,
        "--python_out=../software/src/LogicWeave/proto_gen",
        protobuf_def_path ++ definition_file,
    });
    gen_python_proto_cmd.step.name = "gen-python-proto";
    b.getInstallStep().dependOn(&gen_python_proto_cmd.step);

    // 2. Generate Zig Protobuf code
    const protobuf_dep = b.dependency("protobuf", .{ .target = host_target, .optimize = optimize });
    const protoc_step = protobuf.RunProtocStep.create(b, protobuf_dep.builder, host_target, .{
        .destination_directory = b.path(protobuf_generated_path),
        .source_files = &.{definition_file},
        .include_directories = &.{protobuf_def_path},
    });

    // Make the Zig Protobuf step depend on the Python Protobuf step, to ensure order
    protoc_step.step.dependOn(&gen_python_proto_cmd.step);
    b.getInstallStep().dependOn(&protoc_step.step);

    // 3. Create the Zig module for the generated code
    return b.createModule(.{ .root_source_file = b.path("src/proto_gen/all.pb.zig") });
}

// --- Function to Build the Flashing Executable ---
fn build_flash_firmware(b: *Build, optimize: std.builtin.OptimizeMode, target: std.Build.ResolvedTarget, protbuf_defs: *Module) void {
    const exe = b.addExecutable(.{
        .name = "FlashFirmware",
        .root_source_file = b.path("src/flash.zig"),
        .target = target, // host target
        .optimize = optimize,
    });

    const pb_host_dep = b.dependency("protobuf", .{
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.addImport("protobuf", pb_host_dep.module("protobuf"));
    exe.root_module.addImport("protocol", protbuf_defs);

    const serial = b.dependency("serial", .{});
    exe.root_module.addImport("serial", serial.module("serial"));

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());

    const firmware_name = "main.uf2";
    const run_step = b.step("flash", std.fmt.allocPrint(b.allocator, "Flash the {s} firmware", .{firmware_name}) catch @panic("Failed to allocate flash step description"));
    run_step.dependOn(&run_cmd.step);
}

fn addOptions(b: *Build, fw_options: *Build.Step.Options) void {
    const commit_hash = b.option([]const u8, "GIT_HASH", "The git commit hash") orelse blk: {
        const stdout = b.run(&.{ "git", "rev-parse", "--short=8", "HEAD" });
        break :blk stdout[0 .. stdout.len - 1]; // remove the \n
    };
    const updated_at = b.run(&.{ "git", "log", "-1", "--format=%cd" });

    // ✨ Create and add firmware-specific options dynamically ✨
    fw_options.addOption([]const u8, "GIT_HASH", commit_hash);
    fw_options.addOption([]const u8, "UPDATED_AT", updated_at);
    // Use the `generic` field directly from the current example
    fw_options.addOption(bool, "GENERIC", false);
}

// --- Main Build Function ---
pub fn build(b: *Build) void {
    // 1) Standard build options
    const optimize = .ReleaseFast;
    const host_target = b.standardTargetOptions(.{});

    // 2) Consolidate protobuf handling into a single function
    const protbuf_defs = add_protobuf_handling(b, optimize, host_target);

    const protobuf_dep = b.dependency("protobuf", .{ .target = host_target, .optimize = optimize });

    // 3) Init MicroZig for RP2xxx (e.g. Pico)
    const mz_dep = b.dependency("microzig", .{});
    const mb = microzig.MicroBuild(.{ .rp2xxx = true }).init(b, mz_dep) orelse @panic("MicroZig port not available. Fetch dependencies.");

    // 4) Define Firmware Examples/Targets
    const examples = [_]Example{
        .{
            .name = "logicweave_generic_pico2_arm",
            .target = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
            .file = "src/examples/default.zig",
            .generic = true,
        },
        .{
            .name = "logicweave_generic_pico_arm",
            .target = mb.ports.rp2xxx.boards.raspberrypi.pico,
            .file = "src/examples/default.zig",
            .generic = true,
        },
        .{
            .name = "logicweave_core",
            .target = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
            .file = "src/examples/core.zig",
            .generic = false,
        },
    };

    // 5) Build loop for firmware examples
    for (examples) |ex| {
        const lw_mod = b.addModule("logicweave", .{ .root_source_file = b.path("src/logicweave.zig") });

        // Add the imports into my logicweave module
        lw_mod.addImport("protobuf", protobuf_dep.module("protobuf"));
        lw_mod.addImport("protocol", protbuf_defs);

        // Add the auto generated commit/hash options
        const fw_options = b.addOptions();
        addOptions(b, fw_options);
        lw_mod.addOptions("firmware_config", fw_options);

        // a) Create firmware executable step
        const fw = mb.add_firmware(.{
            .name = ex.name,
            .target = ex.target,
            .optimize = .ReleaseFast,
            .root_source_file = b.path(ex.file),
        });

        // b) Import logicweave
        fw.add_app_import("logicweave", lw_mod, .{ .depend_on_microzig = true });

        // f) Install firmware artifacts
        mb.install_firmware(fw, .{});
    }

    // 6) Host-side flashing tool
    //build_flash_firmware(b, optimize, host_target, protbuf_defs);
}
