// build.zig
const std = @import("std");
const microzig = @import("microzig");
const protobuf = @import("protobuf");
const Build = std.Build;
const Module = Build.Module;

// Use the definition provided by the user
const Target = microzig.Target;
const HardwareAbstractionLayer = microzig.HardwareAbstractionLayer;
// const ModuleImports = microzig.ModuleImports; // Not used directly here, but keep if needed elsewhere

const Example = struct {
    target: *const Target, // Use the provided Target type
    name: []const u8,
    file: []const u8,
};

// --- Function to Build the Flashing Executable (MODIFIED) ---
// This function now includes the protobuf dependency for the host target
fn build_flash_firmware(b: *Build, optimize: std.builtin.OptimizeMode, target: std.Build.ResolvedTarget) void {
    // ... (potential other code you might have here) ...

    const exe = b.addExecutable(.{
        .name = "FlashFirmware",
        .root_source_file = b.path("src/flash.zig"),
        .target = target, // host target
        .optimize = optimize,
    });

    // *** ADDED/MODIFIED SECTION ***
    // Add Protobuf dependency for the host target so the build system
    // can resolve @import("protobuf") if it analyzes example.pb.zig
    // during this step's dependency resolution.
    const pb_host_dep = b.dependency("protobuf", .{
        .target = target, // Use the host target passed to the function
        .optimize = optimize,
    });
    // Add the import to the executable's root module
    exe.root_module.addImport("protobuf", pb_host_dep.module("protobuf"));
    // *** END OF ADDED/MODIFIED SECTION ***

    const serial = b.dependency("serial", .{});
    exe.root_module.addImport("serial", serial.module("serial"));

    // Install the artifact
    b.installArtifact(exe);

    // Set up the run command
    const run_cmd = b.addRunArtifact(exe);
    // Ensure flashing tool build depends on the main install step (which depends on protoc)
    run_cmd.step.dependOn(b.getInstallStep());

    // Define the firmware file name to flash
    const firmware_name = "main.uf2"; // Assuming you want to flash the main firmware
    // Create the "flash" step
    const run_step = b.step("flash", std.fmt.allocPrint(b.allocator, "Flash the {s} firmware", .{firmware_name}) catch @panic("Failed to allocate flash step description"));
    run_step.dependOn(&run_cmd.step); // The "flash" step depends on running the flasher executable
}

// --- Main Build Function (Unchanged Structurally) ---
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
        .destination_directory = b.path("src"),
        .source_files = &.{"all.proto"}, // Change this
        .include_directories = &.{"../proto"}, // Change this
    });

    // Python Protobuf generation step
    const gen_python_proto = b.addSystemCommand(&.{
        "protoc",
        "-I=../proto",
        "--python_out=../software/src/LogicWeave/proto_gen",
        "../proto/all.proto",
    });
    gen_python_proto.step.name = "gen-python-proto";
    b.getInstallStep().dependOn(&gen_python_proto.step);

    // Make sure protoc runs before the final install (and thus before a normal build)
    b.getInstallStep().dependOn(&protoc_step.step);

    // 3) Zig‐module for your generated .pb.zig files
    const proto_module = b.createModule(.{
        .root_source_file = b.path("src/all.pb.zig"), // Ensure this points to a generated file
    });

    // 4) Init MicroZig for RP2xxx (e.g. Pico)
    const mz_dep = b.dependency("microzig", .{});

    // Select the necessary port(s) for MicroBuild
    const mb = microzig.MicroBuild(.{ .rp2xxx = true }).init(b, mz_dep) orelse @panic("MicroZig port not available. Fetch dependencies."); // Use panic or return depending on preference

    // 5) Define Firmware Examples/Targets
    const examples = [_]Example{
        // .{ .name = "main_pico", .target = mb.ports.rp2xxx.boards.raspberrypi.pico, .file = "src/main.zig" },
        .{ .name = "main_pico2_arm", .target = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm, .file = "src/main.zig" },
    };

    // 6) Build loop for firmware examples
    for (examples) |ex| {
        // a) Create firmware executable step
        const fw = mb.add_firmware(.{
            .name = ex.name,
            .target = ex.target,
            .optimize = optimize,
            .root_source_file = b.path(ex.file),
        });

        const commit_hash = b.option([]const u8, "GIT_HASH", "The git commit hash") orelse blk: {
            const stdout = b.run(&.{ "git", "rev-parse", "--short=8", "HEAD" });
            break :blk stdout[0 .. stdout.len - 1]; // remove the \n
        };

        const updated_at = b.run(&.{ "git", "log", "-1", "--format=%cd" });

        // This is also CORRECT. It captures a reference to the future output file.

        const firmware_options = b.addOptions();
        firmware_options.addOption([]const u8, "GIT_HASH", commit_hash);
        firmware_options.addOption([]const u8, "UPDATED_AT", updated_at);

        fw.add_options("firmware_config", firmware_options);

        // b) Import your generated protocol definitions into the firmware application
        fw.add_app_import("protocol", proto_module, .{});

        // c) Build the protobuf *runtime* library for the firmware's target
        const zig_tgt = b.resolveTargetQuery(ex.target.zig_target);
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

    // 7) Host‑side flashing tool (Calls the modified helper function)
    build_flash_firmware(b, optimize, host_target);
}
