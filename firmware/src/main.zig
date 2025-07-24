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

    try hardware.sd.initialize(20_000_000);
    rp2xxx.time.sleep_ms(100);

    // tell ZFAT about our physical disk:
    fatfs.disks[0] = &hardware.sd.interface;

    try hardware.global_fs.mount("0:", true);
    defer fatfs.FileSystem.unmount("0:") catch |e| std.log.err("failed to unmount filesystem: {s}", .{@errorName(e)});

    var dir = try fatfs.Dir.open("0:/");
    defer dir.close();

    std.log.info("Files in root directory:", .{});

    // Iterate through the directory contents
    while (try dir.next()) |file_info| {
        // file_info.fname is a null-terminated C array ([:0]u8)
        // You might need to convert it to a Zig slice or use std.mem.span
        std.log.info("- {s}", .{file_info.name()});
    }

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
