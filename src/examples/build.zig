const std = @import("std");
const microzig = @import("microzig");
const Build = std.Build;
const Module = Build.Module;
// Use the definition provided by the user
const Target = microzig.Target;

const MicroBuild = microzig.MicroBuild(.{
    .rp2xxx = true,
});

const PicoTarget = enum {
    RP2040,
    RP2350,
};

const Example = struct {
    target: PicoTarget,
    name: []const u8,
    file: []const u8,
};

// 4) Define Firmware Examples/Targets
const examples = [_]Example{
    .{
        .name = "lw_pico2",
        .target = .RP2350,
        .file = "src/logicweave.zig",
    },
    .{
        .name = "lw_core",
        .target = .RP2350,
        .file = "src/logicweave_core.zig",
    },
    .{
        .name = "lw_pico",
        .target = .RP2040,
        .file = "src/logicweave.zig",
    },
};

// --- Main Build Function ---
pub fn build(b: *Build) void {
    // 1. Define and get the example name from a CLI option
    const example_name_option = b.option([]const u8, "example-name", "The name of the example to build (e.g., 'logicweave_pico')");

    // Check if the user provided the option
    const example_name = if (example_name_option) |name| name else {
        std.debug.print("Error: Please provide an example name using -Dexample-name=<name>\n", .{});
        std.debug.print("Available examples: {s}\n", .{@as([]const u8, examples[0].name)});
        // Optionally, you might want to print all available examples or exit gracefully
        return; // Halt the build process
    };

    // 2. Find the example matching the provided name
    var found_example: ?Example = null;
    for (examples) |ex| {
        if (std.mem.eql(u8, ex.name, example_name)) {
            found_example = ex;
            break;
        }
    }

    const ex = found_example orelse {
        std.debug.print("Error: Example '{s}' not found.\n", .{example_name});
        return; // Halt the build process
    };

    // --- Steps 3 and 4: Dependencies and Building the Single Firmware ---

    const lw_dep = b.dependency("logicweave", .{});
    const mz_dep = b.dependency("microzig", .{});

    const mb = MicroBuild.init(b, mz_dep) orelse return;
    const lw_mod = lw_dep.module("logicweave");

    const pb_dep = b.dependency("protobuf", .{});

    // Build only the selected example
    const pico_target = switch (ex.target) {
        .RP2040 => mb.ports.rp2xxx.boards.raspberrypi.pico,
        .RP2350 => mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
    };

    const fw = mb.add_firmware(.{
        .name = ex.name,
        .target = pico_target,
        .board = if (ex.target == .RP2350) .{
            .name = "rp2350b",
            .root_source_file = b.path("rp2350b.zig"),
        } else null,
        .optimize = .ReleaseFast,
        .root_source_file = b.path(ex.file),
    });

    const messages_mod = b.createModule(.{ .root_source_file = b.path("../proto_gen/logicweave.pb.zig") });
    const lw_core_mod = b.createModule(.{ .root_source_file = b.path("../proto_gen/logicweave_core.pb.zig") });
    lw_core_mod.addImport("protobuf", pb_dep.module("protobuf"));

    // Now, just tell the firmware about your logicweave module.
    fw.add_app_import("logicweave", lw_mod, .{ .depend_on_microzig = true });
    fw.add_app_import("protobuf", pb_dep.module("protobuf"), .{});
    fw.add_app_import("lw_core", lw_core_mod, .{});
    fw.add_app_import("lw_standard", messages_mod, .{});

    // Create an install step for the firmware and capture the artifact object.
    const fw_install = mb.add_install_firmware(fw, .{});

    const host_target = b.standardTargetOptions(.{});
    const flash_mod = b.addModule("flash", .{
        .root_source_file = b.path("tools/flash.zig"),
        .target = host_target,
    });

    const speed_test_mod = b.addModule("speed", .{
        .root_source_file = b.path("tools/speed_test.zig"),
        .target = host_target,
        .optimize = .ReleaseFast,
    });

    // 1. Build the 'flash.zig' utility as an executable
    const speed_test_tool = b.addExecutable(.{
        .name = "speed",
        .root_module = speed_test_mod,
    });

    speed_test_tool.root_module.addImport("protobuf", pb_dep.module("protobuf"));
    speed_test_tool.root_module.addImport("messages", messages_mod);

    // 1. Build the 'flash.zig' utility as an executable
    const flash_tool = b.addExecutable(.{
        .name = "flash",
        .root_module = flash_mod,
    });

    flash_tool.root_module.addImport("protobuf", pb_dep.module("protobuf"));

    messages_mod.addImport("protobuf", pb_dep.module("protobuf"));

    flash_tool.root_module.addImport("messages", messages_mod);

    const serial = b.dependency("serial", .{});
    flash_tool.root_module.addImport("serial", serial.module("serial"));
    speed_test_tool.root_module.addImport("serial", serial.module("serial"));

    b.installArtifact(flash_tool);
    b.installArtifact(speed_test_tool);

    const speed_test_step = b.step("speed", "Builds the firmware and runs the speed test utility.");

    const speed_command = b.addRunArtifact(speed_test_tool);
    speed_command.step.dependOn(&speed_test_tool.step); // Depend on the tool's build
    speed_command.step.dependOn(&fw_install.step); // **NEW:** Depend on the firmware install

    speed_test_step.dependOn(&speed_command.step);

    // 2. Create the 'flash' step
    const flash_step = b.step("flash", "Builds and flashes the selected firmware to the target device.");

    // 3. Create the 'RunStep' that executes the flash tool
    const flash_command = b.addRunArtifact(flash_tool);

    // Set the executable to be the compiled 'flash_tool'
    flash_command.step.dependOn(&flash_tool.step);
    flash_command.step.dependOn(&fw_install.step);

    // Add the firmware output file as an argument to the flash tool.
    // By passing the `fw_install` artifact directly, Zig's build system
    // understands the dependency and will substitute the correct path at runtime.
    // This makes any other explicit `dependOn` for the firmware step redundant.
    flash_command.addArg("--file");
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    const file_path = std.mem.concat(allocator, u8, &[_][]const u8{ "zig-out/firmware/", ex.name, ".uf2" }) catch "zig-out/firmware/logicweave_pico2_arm.uf2";
    flash_command.addArg(file_path);

    // Add the run command to the flash step
    flash_step.dependOn(&flash_command.step);
}
