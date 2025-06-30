// src/main.zig
const std = @import("std");
const microzig = @import("microzig");

// Import our new modules
const hardware = @import("hardware.zig");
const usb_cfg = @import("usb_config.zig");
const protocol_handler = @import("protocol_handler.zig");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const usb = rp2xxx.usb;

pub const microzig_options = microzig.Options{
    .log_level = .debug,
    .logFn = rp2xxx.uart.logFn,
    .cpu = .{ .ram_vectors = true },
};

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.log.err("panic: {s}", .{message});
    @breakpoint();
    rp2xxx.rom.reset_usb_boot(0, 0);
    while (true) {}
}

pub fn main() !void {
    // 1. Initialize all hardware
    hardware.init() catch |err| {
        std.log.err("Hardware init failed: {}", .{err});
        // Handle failure, maybe blink an LED
        while (true) {}
    };

    // 2. Initialize the USB device
    const usb_dev = usb.Usb(.{});
    usb_dev.init_clk();
    usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

    // 3. Setup allocator for protocol handler
    var buffer: [128]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
    const allocator = fba.allocator();

    var last_update_time: u64 = time.get_time_since_boot().to_us();

    // 4. Main loop
    while (true) {
        // Poll for USB events
        usb_dev.task(false) catch unreachable;

        // Check for and handle any incoming USB commands
        protocol_handler.handle_incoming_usb(allocator);

        // Periodically update the e-paper display
        const now = time.get_time_since_boot().to_us();
        if (now - last_update_time > 1_000_000) { // 1 second
            last_update_time = now;
            hardware.poll_and_update_display() catch {}; // Ignore display errors
        }
    }
}
