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

// --- Function to Build the Flashing Executable ---
fn build_flash_firmware(b: *Build, optimize: std.builtin.OptimizeMode, target: std.Build.ResolvedTarget) void {
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

    const serial = b.dependency("serial", .{});
    exe.root_module.addImport("serial", serial.module("serial"));

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());

    const firmware_name = "main.uf2";
    const run_step = b.step("flash", std.fmt.allocPrint(b.allocator, "Flash the {s} firmware", .{firmware_name}) catch @panic("Failed to allocate flash step description"));
    run_step.dependOn(&run_cmd.step);
}

fn gen_python_proto(b: *Build) void {
    // Python Protobuf generation step
    const gen_python_proto_cmd = b.addSystemCommand(&.{
        "protoc",
        "-I=../proto",
        "--python_out=../software/src/LogicWeave/proto_gen",
        "../proto/all.proto",
    });
    gen_python_proto_cmd.step.name = "gen-python-proto";
    b.getInstallStep().dependOn(&gen_python_proto_cmd.step);
}

// --- Main Build Function ---
pub fn build(b: *Build) void {
    // 1) Standard build options
    const optimize = .ReleaseFast;
    const host_target = b.standardTargetOptions(.{});
    gen_python_proto(b);

    const protobuf_dep = b.dependency("protobuf", .{
        .target = host_target,
        .optimize = optimize,
    });

    const protoc_step = protobuf.RunProtocStep.create(b, protobuf_dep.builder, host_target, .{
        .destination_directory = b.path("src/proto_gen"),
        .source_files = &.{"all.proto"},
        .include_directories = &.{"../proto"},
    });

    b.getInstallStep().dependOn(&protoc_step.step);

    const protbuf_defs = b.createModule(.{ .root_source_file = b.path("src/proto_gen/all.pb.zig") });

    // 4) Init MicroZig for RP2xxx (e.g. Pico)
    const mz_dep = b.dependency("microzig", .{});
    const mb = microzig.MicroBuild(.{ .rp2xxx = true }).init(b, mz_dep) orelse @panic("MicroZig port not available. Fetch dependencies.");

    const lw_mod = b.addModule("logicweave", .{ .root_source_file = b.path("src/logicweave.zig") });
    lw_mod.addImport("protobuf", protobuf_dep.module("protobuf"));
    lw_mod.addImport("protocol", protbuf_defs);

    const commit_hash = b.option([]const u8, "GIT_HASH", "The git commit hash") orelse blk: {
        const stdout = b.run(&.{ "git", "rev-parse", "--short=8", "HEAD" });
        break :blk stdout[0 .. stdout.len - 1]; // remove the \n
    };
    const updated_at = b.run(&.{ "git", "log", "-1", "--format=%cd" });

    // ✨ Create and add firmware-specific options dynamically ✨
    const fw_options = b.addOptions();
    fw_options.addOption([]const u8, "GIT_HASH", commit_hash);
    fw_options.addOption([]const u8, "UPDATED_AT", updated_at);
    // Use the `generic` field directly from the current example
    fw_options.addOption(bool, "GENERIC", false);

    lw_mod.addOptions("firmware_config", fw_options);

    // 5) Define Firmware Examples/Targets
    const examples = [_]Example{
        .{
            .name = "logicweave_generic_pico2_arm",
            .target = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
            .file = "src/examples/core.zig",
            .generic = true,
        },
    };

    // 6) Build loop for firmware examples
    for (examples) |ex| {
        // a) Create firmware executable step
        const fw = mb.add_firmware(.{
            .name = ex.name,
            .target = ex.target,
            .optimize = .ReleaseFast,
            .root_source_file = b.path(ex.file),
        });

        // b) Import your generated protocol definitions into the firmware application
        //fw.add_app_import("protocol", proto_module, .{});
        fw.add_app_import("logicweave", lw_mod, .{ .depend_on_microzig = true });

        const zig_tgt = b.resolveTargetQuery(ex.target.zig_target);

        // c) Build the protobuf *runtime* library for the firmware's target
        const pb_fw_dep = b.dependency("protobuf", .{ .target = zig_tgt, .optimize = optimize });
        const pb_fw_mod = pb_fw_dep.module("protobuf");

        // d) Import the protobuf runtime into the firmware application
        fw.add_app_import("protobuf", pb_fw_mod, .{});
        fw.add_options("firmware_config", fw_options);

        // f) Install firmware artifacts
        mb.install_firmware(fw, .{});
    }

    // 7) Host‑side flashing tool
    build_flash_firmware(b, optimize, host_target);
}
