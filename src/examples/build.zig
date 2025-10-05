const std = @import("std");
const microzig = @import("microzig");
const Build = std.Build;
const Module = Build.Module;

// Use the definition provided by the user
const Target = microzig.Target;

const MicroBuild = microzig.MicroBuild(.{
    .rp2xxx = true,
});

const Example = struct {
    target: *const Target,
    name: []const u8,
    file: []const u8,
};

// --- Main Build Function ---
pub fn build(b: *Build) void {
    // 1) Standard build options
    const lw_dep = b.dependency("logicweave", .{});
    const lw_mod = lw_dep.module("logicweave");

    // 3) Init MicroZig for RP2xxx (e.g. Pico)
    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;

    // 4) Define Firmware Examples/Targets
    const examples = [_]Example{
        .{
            .name = "logicweave_pico2_arm",
            .target = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm,
            .file = "src/pico.zig",
        },
    };

    // 5) Build loop for firmware examples
    for (examples) |ex| {
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
}
