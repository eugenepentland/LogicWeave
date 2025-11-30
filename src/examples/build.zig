const std = @import("std");
const microzig = @import("microzig");
const Build = std.Build;

// Use the definition provided by the user
const MicroBuild = microzig.MicroBuild(.{
    .rp2xxx = true,
});

const PicoTarget = enum {
    RP2040,
    RP2350,
    RP2350B,
};

const Example = struct {
    target: PicoTarget,
    name: []const u8,
    file: []const u8,
};

// Define Firmware Examples/Targets
const examples = [_]Example{
    .{
        .name = "lw_rp2350b",
        .target = .RP2350B,
        .file = "src/logicweave.zig",
    },
    .{
        .name = "lw_rp2350",
        .target = .RP2350,
        .file = "src/logicweave.zig",
    },
    .{
        .name = "lw_core",
        .target = .RP2350B,
        .file = "src/logicweave_core.zig",
    },
    .{
        .name = "lw_rp2040",
        .target = .RP2040,
        .file = "src/logicweave.zig",
    },
};

pub fn build(b: *Build) void {
    // 1. CLI Option: Get example name
    const example_name_option = b.option([]const u8, "example-name", "The name of the example to build (e.g., 'lw_pico')");

    const example_name = if (example_name_option) |name| name else {
        std.debug.print("Error: Please provide an example name using -Dexample-name=<name>\n", .{});
        std.debug.print("Available examples: {s}\n", .{@as([]const u8, examples[0].name)});
        return;
    };

    // 2. Find the example
    var found_example: ?Example = null;
    for (examples) |ex| {
        if (std.mem.eql(u8, ex.name, example_name)) {
            found_example = ex;
            break;
        }
    }

    const ex = found_example orelse {
        std.debug.print("Error: Example '{s}' not found.\n", .{example_name});
        return;
    };

    // --- Dependencies ---
    const lw_dep = b.dependency("logicweave", .{});
    const mz_dep = b.dependency("microzig", .{});
    const pb_dep = b.dependency("protobuf", .{});
    const serial_dep = b.dependency("serial", .{});

    // --- MicroZig Setup ---
    const mb = MicroBuild.init(b, mz_dep) orelse return;
    const lw_mod = lw_dep.module("logicweave");

    const pico_target = switch (ex.target) {
        .RP2040 => mb.ports.rp2xxx.boards.raspberrypi.pico,
        .RP2350, .RP2350B => mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
    };

    // --- Create Firmware ---
    const fw = mb.add_firmware(.{
        .name = ex.name,
        .target = pico_target,
        .board = if (ex.target == .RP2350B) .{
            .name = "rp2350b",
            .root_source_file = b.path("rp2350b.zig"),
        } else null,
        .optimize = .ReleaseFast,
        .root_source_file = b.path(ex.file),
    });

    // --- Protocol Buffers / LogicWeave Modules ---
    const messages_mod = b.createModule(.{ .root_source_file = b.path("../proto_gen/logicweave.pb.zig") });
    messages_mod.addImport("protobuf", pb_dep.module("protobuf"));

    const lw_core_mod = b.createModule(.{ .root_source_file = b.path("../proto_gen/logicweave_core.pb.zig") });
    lw_core_mod.addImport("protobuf", pb_dep.module("protobuf"));

    // --- Firmware Imports ---
    fw.add_app_import("logicweave", lw_mod, .{ .depend_on_microzig = true });
    fw.add_app_import("protobuf", pb_dep.module("protobuf"), .{});
    fw.add_app_import("lw_core", lw_core_mod, .{});
    fw.add_app_import("lw_standard", messages_mod, .{});

    // --- Install Firmware ---
    // This creates the logic to put the UF2 in zig-out/firmware/
    const fw_install = mb.add_install_firmware(fw, .{});

    // **NEW STEP**: "firmware"
    // Run `zig build firmware -Dexample-name=...` to build without flashing
    const firmware_step = b.step("firmware", "Build the firmware only (no flashing)");
    firmware_step.dependOn(&fw_install.step);

    // Make the default `zig build` also just build the firmware
    b.getInstallStep().dependOn(&fw_install.step);

    // --- Host Tools (Flash Utility) ---
    const host_target = b.standardTargetOptions(.{});
    
    const flash_mod = b.addModule("flash", .{
        .root_source_file = b.path("tools/flash.zig"),
        .target = host_target,
    });

    const flash_tool = b.addExecutable(.{
        .name = "flash",
        .root_module = flash_mod,
    });

    // Flash Tool Imports
    flash_tool.root_module.addImport("protobuf", pb_dep.module("protobuf"));
    flash_tool.root_module.addImport("messages", messages_mod);
    flash_tool.root_module.addImport("serial", serial_dep.module("serial"));

    b.installArtifact(flash_tool);

    // --- Flash Step ---
    const flash_step = b.step("flash", "Builds and flashes the selected firmware");
    const flash_command = b.addRunArtifact(flash_tool);

    // Ensure we build the firmware and the tool before running the flash command
    flash_command.step.dependOn(&flash_tool.step);
    flash_command.step.dependOn(&fw_install.step);

    // Arguments for the flash tool
    flash_command.addArg("--file");
    
    // Use b.fmt to cleaner string concatenation for the path
    const file_path = b.fmt("zig-out/firmware/{s}.uf2", .{ex.name});
    flash_command.addArg(file_path);

    flash_step.dependOn(&flash_command.step);
}