// src/main.zig
const std = @import("std");
const microzig = @import("microzig");
const Duration = microzig.drivers.time.Duration;
const firmware_config = @import("firmware_config");
const fatfs = @import("zfat");
const menu = @import("menu.zig");
const Graphics = @import("graphics.zig");
const protobuf = @import("protobuf");

// Import our new modules
const hardware = @import("hardware.zig");
const usb_cfg = @import("usb_config.zig");
const protocol_handler = @import("protocol_handler.zig");
const definitions = @import("proto_gen/all.pb.zig");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const usb = rp2xxx.usb;
var writer: std.ArrayList(u8) = undefined;
var ok_msg: definitions.AppMessage = undefined;
pub const microzig_options = microzig.Options{
    .log_level = .info,
    .logFn = rp2xxx.uart.logFn,
    .cpu = .{ .ram_vectors = true },
};

pub fn panic(message: []const u8, s: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.log.err("The RP2350 has crashed: {s}. {any}", .{ message, s });
    @breakpoint();
    rp2xxx.rom.reboot_to_bootsel_now() catch {};
    while (true) {}
}

fn core1() void {
    // 1. Setup allocator for protocol handler
    var buffer: [256]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
    const allocator = fba.allocator();
    writer = std.ArrayList(u8).init(allocator);

    while (true) {
        // Wait for usb data length in the fifo
        const message_len = rp2xxx.multicore.fifo.read_blocking();

        // Get the slice to the data
        shared_data_spinlock.lock();
        const incoming_data = shared_usb_rx_buff[0..message_len];
        shared_data_spinlock.unlock();

        // Get the response
        protocol_handler.handle_incoming_usb(allocator, &writer, incoming_data) catch |err| {
            var err_buff: [64]u8 = undefined;
            const formatted_err = std.fmt.bufPrint(err_buff[0..], "{}", .{err}) catch "format error";
            protocol_handler.usb_cdc_write_protobuf(.{ .error_response = .{ .message = protobuf.ManagedString.managed(formatted_err) } }, &writer) catch {};
        };

        // Send the response
        shared_data_spinlock.lock();
        //std.mem.copyForwards(u8, &shared_usb_tx_buff, writer.items);
        rp2xxx.multicore.fifo.write_blocking(@intCast(writer.items.len));
        shared_data_spinlock.unlock();
    }
}

var stack: [8192]u32 = undefined;

pub fn main() !void {
    var buffer: [4048]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
    const allocator = fba.allocator();

    // Initalize the screen, USB PD ports, intterupts for logicweave board
    if (comptime !firmware_config.GENERIC) {
        hardware.init(allocator) catch |err| {
            std.log.err("Hardware init failed: {}", .{err});
        };
        //try menu.render_menu();
    }
    rp2xxx.multicore.launch_core1_with_stack(&core1, &stack);

    // 2. Initialize the USB device
    const usb_dev = usb.Usb(.{});
    usb_dev.init_clk();
    usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

    // 4. Main loop
    while (true) {
        // Poll for USB events
        usb_dev.task(false) catch unreachable;

        handleUsbRx();
        handleUsbTx();
    }
}

fn handleUsbRx() void {
    // Read in any USB data if there is any
    const rx_data = usb_cdc_read();
    if (rx_data.len > 0) {
        // Copy the data to the shared memory
        shared_data_spinlock.lock();
        std.mem.copyForwards(u8, &shared_usb_rx_buff, rx_data);
        shared_data_spinlock.unlock();

        // Signal to core1 data is ready and what its length it
        rp2xxx.multicore.fifo.write_blocking(@intCast(rx_data.len));
    }
}

fn handleUsbTx() void {
    // Writes a response to the fifo if it gets any
    const response_len = rp2xxx.multicore.fifo.read();
    if (response_len) |_| {
        shared_data_spinlock.lock();
        const tx_data = writer.items;
        usb_cdc_write(tx_data);
        shared_data_spinlock.unlock();
    }
}

var shared_usb_rx_buff: [128]u8 = undefined;
var shared_usb_tx_buff: [128]u8 = undefined;
var usb_rx_buff: [128]u8 = undefined;
var usb_tx_buff: [128]u8 = undefined;

const shared_data_spinlock = rp2xxx.multicore.Spinlock.init(0);

// --- USB Communication Functions ---
fn usb_cdc_read() []const u8 {
    var total_read: usize = 0;
    var read_buff: []u8 = usb_rx_buff[0..];

    while (true) {
        const len = usb_cfg.driver_cdc.read(read_buff);
        read_buff = read_buff[len..];
        total_read += len;
        if (len == 0) break;
    }
    return usb_rx_buff[0..total_read];
}

fn usb_cdc_write(buff: []const u8) void {
    const usb_dev = rp2xxx.usb.Usb(.{});
    var write_buff = buff;

    const msg_lenth: u8 = @intCast(write_buff.len);
    _ = usb_cfg.driver_cdc.write(&[_]u8{msg_lenth});

    while (write_buff.len > 0) {
        write_buff = usb_cfg.driver_cdc.write(write_buff);
        usb_dev.task(false) catch unreachable;
    }
    _ = usb_cfg.driver_cdc.write_flush();
    usb_dev.task(false) catch unreachable;
}
