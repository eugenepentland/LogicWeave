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

// --- Main Build Function ---
pub fn build(b: *Build) void {
    // 1) Standard build options
    const optimize = b.standardOptimizeOption(.{});
    const host_target = b.standardTargetOptions(.{});

    // 2) Host‑side: protoc generation step
    const pb_host_dep = b.dependency("protobuf", .{
        .target = host_target,
        .optimize = optimize,
    });
    const protoc_step = protobuf.RunProtocStep.create(b, pb_host_dep.builder, host_target, .{
        .destination_directory = b.path("src/proto_gen"),
        .source_files = &.{"all.proto"},
        .include_directories = &.{"../proto"},
    });
    b.getInstallStep().dependOn(&protoc_step.step);

    // Python Protobuf generation step
    const gen_python_proto = b.addSystemCommand(&.{
        "protoc",
        "-I=../proto",
        "--python_out=../software/src/LogicWeave/proto_gen",
        "../proto/all.proto",
    });
    gen_python_proto.step.name = "gen-python-proto";
    b.getInstallStep().dependOn(&gen_python_proto.step);

    // 3) Zig‐module for your generated .pb.zig files
    const proto_module = b.createModule(.{
        .root_source_file = b.path("src/proto_gen/all.pb.zig"),
    });

    // 4) Init MicroZig for RP2xxx (e.g. Pico)
    const mz_dep = b.dependency("microzig", .{});
    const mb = microzig.MicroBuild(.{ .rp2xxx = true }).init(b, mz_dep) orelse @panic("MicroZig port not available. Fetch dependencies.");

    // 5) Define Firmware Examples/Targets
    const examples = [_]Example{
        .{
            .name = "logicweave_pico2_arm",
            .target = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
            .file = "src/main.zig",
            .generic = false,
        },
        .{
            .name = "logicweave_generic_pico2_arm",
            .target = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
            .file = "src/main.zig",
            .generic = true,
        },
        .{
            .name = "logicweave_generic_pico_arm",
            .target = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
            .file = "src/main.zig",
            .generic = true,
        },
    };

    const commit_hash = b.option([]const u8, "GIT_HASH", "The git commit hash") orelse blk: {
        const stdout = b.run(&.{ "git", "rev-parse", "--short=8", "HEAD" });
        break :blk stdout[0 .. stdout.len - 1]; // remove the \n
    };
    const updated_at = b.run(&.{ "git", "log", "-1", "--format=%cd" });

    // 6) Build loop for firmware examples
    for (examples) |ex| {
        // a) Create firmware executable step
        const fw = mb.add_firmware(.{
            .name = ex.name,
            .target = ex.target,
            .optimize = optimize,
            .root_source_file = b.path(ex.file),
        });

        // ✨ Create and add firmware-specific options dynamically ✨
        const fw_options = b.addOptions();
        fw_options.addOption([]const u8, "GIT_HASH", commit_hash);
        fw_options.addOption([]const u8, "UPDATED_AT", updated_at);
        // Use the `generic` field directly from the current example
        fw_options.addOption(bool, "GENERIC", ex.generic);

        fw.add_options("firmware_config", fw_options);

        // b) Import your generated protocol definitions into the firmware application
        fw.add_app_import("protocol", proto_module, .{});

        const zig_tgt = b.resolveTargetQuery(ex.target.zig_target);

        const zfat_dep = b.dependency("zfat", .{
            .target = zig_tgt,
            .optimize = optimize,
            .code_page = .us,
            .@"sector-size" = @as(u32, 512),
            .@"volume-count" = @as(u32, 1),
            // .@"volume-names" = @as([]const u8, "a,b,c,h,z"), // TODO(fqu): Requires VolToPart to be defined
            .@"no-libc" = true,
            // Enable features:
            .find = true,
            .mkfs = true,
            .fastseek = true,
            .expand = true,
            .chmod = true,
            .label = true,
            .reentrant = false,
            .forward = true,
            .relative_path_api = .enabled_with_getcwd,
            // .multi_partition = true, // TODO(fqu): Requires VolToPart to be defined
            .lba64 = true,
            .use_trim = true,
            .exfat = true,
            .@"static-rtc" = @as([]const u8, "2025-07-22"),
        });

        const zfat_mod = zfat_dep.module("zfat");

        // Add Foundation Libc and get its paths
        const libc_dep = b.dependency("foundation_libc", .{
            .target = zig_tgt,
            .optimize = optimize,
            .single_threaded = true,
        });

        const libc = libc_dep.artifact("foundation");
        const tree = libc.getEmittedIncludeTree();

        //zfat_mod.addIncludePath(tree);
        zfat_mod.link_objects.items[0].other_step.root_module.addIncludePath(tree);
        fw.artifact.linkLibrary(libc);

        fw.add_app_import("zfat", zfat_mod, .{});

        // c) Build the protobuf *runtime* library for the firmware's target
        const pb_fw_dep = b.dependency("protobuf", .{
            .target = zig_tgt,
            .optimize = optimize,
        });
        const pb_fw_mod = pb_fw_dep.module("protobuf");

        // d) Import the protobuf runtime into the firmware application
        fw.add_app_import("protobuf", pb_fw_mod, .{});

        // f) Install firmware artifacts
        mb.install_firmware(fw, .{});
    }

    // 7) Host‑side flashing tool
    build_flash_firmware(b, optimize, host_target);
}
