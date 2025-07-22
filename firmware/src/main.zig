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
    const uart = rp2xxx.uart.instance.num(0);
    const baud_rate = 115200;
    const uart_tx_pin = rp2xxx.gpio.num(32);
    uart_tx_pin.set_function(.uart);

    uart.apply(.{
        .baud_rate = baud_rate,
        .clock_config = rp2xxx.clock_config,
    });

    rp2xxx.uart.init_logger(uart);

    std.log.info("booting up", .{});
    // 1. Setup allocator for protocol handler
    var buffer: [2048]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
    const allocator = fba.allocator();

    // Initalize the screen, USB PD ports, intterupts for logicweave board
    if (comptime !firmware_config.GENERIC) {
        hardware.init(allocator) catch |err| {
            std.log.err("Hardware init failed: {}", .{err});
        };
        hardware.poll_and_update_display() catch {}; // Ignore display errors
    }

    try hardware.sd.initialize(20_000_000);

    std.log.info("Mounting FS", .{});
    try hardware.global_fs.mount("0:", true);
    defer fatfs.FileSystem.unmount("0:") catch |e| std.log.err("failed to unmount: {s}", .{@errorName(e)});

    // --- Read Root Directory ---
    std.log.info("Reading root directory:", .{});
    // var root_dir = try fatfs.Dir.open("/");
    //defer root_dir.close();

    std.log.info("Finished reading directory.", .{});

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

        if (comptime !firmware_config.GENERIC) {
            // Periodically update the e-paper display
            const now = time.get_time_since_boot().to_us();
            if (now - last_update_time > 2_000_000) {
                last_update_time = now;
                hardware.poll_and_update_display() catch {}; // Ignore display errors
            }
        }
    }
}
