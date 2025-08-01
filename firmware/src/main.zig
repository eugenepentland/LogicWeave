// src/main.zig
const std = @import("std");
const microzig = @import("microzig");
const Duration = microzig.drivers.time.Duration;
const firmware_config = @import("firmware_config");
const fatfs = @import("zfat");

// Import our new modules
const hardware = @import("hardware.zig");
const usb_cfg = @import("usb_config.zig");
const protocol_handler = @import("protocol_handler.zig");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const usb = rp2xxx.usb;

pub const microzig_options = microzig.Options{
    .log_level = .info,
    .logFn = rp2xxx.uart.logFn,
    .cpu = .{ .ram_vectors = true },
};

pub fn panic(message: []const u8, s: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.log.err("The RP2350 has crashed: {s}. {any}", .{ message, s });
    @breakpoint();
    rp2xxx.rom.reset_usb_boot(0, 0);
    while (true) {}
}

fn core1() void {
    //var last_update_time: u64 = time.get_time_since_boot().to_us();
    hardware.pps_init() catch {};
    hardware.screen_init() catch {};

    while (true) {
        hardware.poll_and_update_display() catch {}; // Ignore display errors
    }
}

var stack: [256]u32 = undefined;

pub fn main() !void {
    std.log.info("booting up", .{});
    // 1. Setup allocator for protocol handler
    var buffer: [4096]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
    const allocator = fba.allocator();

    // Initalize the screen, USB PD ports, intterupts for logicweave board
    if (comptime !firmware_config.GENERIC) {
        hardware.init(allocator) catch |err| {
            std.log.err("Hardware init failed: {}", .{err});
        };
        //hardware.poll_and_update_display() catch {}; // Ignore display errors
    }
    rp2xxx.multicore.launch_core1_with_stack(core1, &stack);

    //try hardware.sd.initialize(20_000_000);

    // 2. Initialize the USB device
    const usb_dev = usb.Usb(.{});
    usb_dev.init_clk();
    usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

    // 4. Main loop
    while (true) {
        // Poll for USB events
        usb_dev.task(false) catch unreachable;

        // Check for and handle any incoming USB commands
        protocol_handler.handle_incoming_usb(allocator);

        if (comptime !firmware_config.GENERIC) {
            // Check if the buttons have been pressed
            if (hardware.pps1_btn_pressed) {
                hardware.pps1.toggle();
                hardware.pps1_btn_pressed = false;
            }

            if (hardware.pps2_btn_pressed) {
                hardware.pps2.toggle();
                hardware.pps2_btn_pressed = false;
            }
        }
    }
}
