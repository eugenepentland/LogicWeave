// src/main.zig
const std = @import("std");
const microzig = @import("microzig");
const Duration = microzig.drivers.time.Duration;

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
    // Enable uart logging for debugging
    const uart_tx_pin = rp2xxx.gpio.num(32);
    const uart = rp2xxx.uart.instance.num(0);

    uart_tx_pin.set_function(.uart);

    uart.apply(.{
        .baud_rate = 115200,
        .clock_config = rp2xxx.clock_config,
    });

    rp2xxx.uart.init_logger(uart);

    // 3. Setup allocator for protocol handler
    var buffer: [2048]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
    const allocator = fba.allocator();

    std.log.info("Booting up", .{});

    // 1. Initialize all hardware
    hardware.init(allocator) catch |err| {
        std.log.err("Hardware init failed: {}", .{err});
        // Handle failure, maybe blink an LED
    };

    hardware.sd.initialize(20_000_000) catch |err| {
        std.log.err("SD init failed: {}", .{err});
    };

    const csd = try hardware.sd.read_csd();
    std.log.info("{any}", .{csd});

    const data: [512]u8 = [_]u8{0x00} ** 512;
    try hardware.sd.write_block(0, &data);

    const buff = try hardware.sd.read_block(1, 512);
    std.log.info("sd block 0 {any}", .{buff});

    // 2. Initialize the USB device
    const usb_dev = usb.Usb(.{});
    usb_dev.init_clk();
    usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

    var last_update_time: u64 = time.get_time_since_boot().to_us();

    // 4. Main loop
    while (true) {
        // Poll for USB events
        usb_dev.task(false) catch unreachable;

        // Check for and handle any incoming USB commands
        protocol_handler.handle_incoming_usb(allocator);

        // Periodically update the e-paper display
        const now = time.get_time_since_boot().to_us();
        if (now - last_update_time > 1_000_000_000) { // 1 second
            last_update_time = now;
            //hardware.poll_and_update_display() catch {}; // Ignore display errors
        }
    }
}
