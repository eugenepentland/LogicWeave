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
        .name = "logicweave_pico2_arm",
        .target = .RP2350,
        .file = "src/pico.zig",
    },
    .{
        .name = "logicweave_pico",
        .target = .RP2040,
        .file = "src/pico.zig",
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

    // Build only the selected example
    const target = switch (ex.target) {
        .RP2040 => mb.ports.rp2xxx.boards.raspberrypi.pico,
        .RP2350 => mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
    };

    const fw = mb.add_firmware(.{
        .name = ex.name,
        .target = target,
        .optimize = .ReleaseFast,
        .root_source_file = b.path(ex.file),
    });

    // Now, just tell the firmware about your logicweave module.
    fw.add_app_import("logicweave", lw_mod, .{ .depend_on_microzig = true });

    mb.install_firmware(fw, .{});
}