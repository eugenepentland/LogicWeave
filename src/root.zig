// src/main.zig
const std = @import("std");
const microzig = @import("microzig");
const firmware_config = @import("firmware_config");
pub const protobuf = @import("protobuf");
pub const messages = @import("proto_gen/all.pb.zig");

// Import our new modules
const usb_cfg = @import("usb_config.zig");
const protocol_handler = @import("protocol_handler.zig");
pub const Interrupt = @import("interrupts.zig");

const Duration = microzig.drivers.time.Duration;
const peripherals = microzig.chip.peripherals;
const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const usb = rp2xxx.usb;

pub var usb_writer: std.Io.Writer = .fixed(&usb_tx_buff);
var shared_usb_rx_buff: [128]u8 = undefined;
var shared_usb_tx_buff: [128]u8 = undefined;
var usb_rx_buff: [128]u8 = undefined;
var usb_tx_buff: [128]u8 = undefined;

pub var custom_usb_handler: ?*const fn (std.mem.Allocator, *std.Io.Reader, *std.Io.Writer) void = null;
const shared_data_spinlock = rp2xxx.multicore.Spinlock.init(0);

const LogicWeave = @This();

fn panic(message: []const u8, s: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.log.err("The RP2350 has crashed: {s}. {any}", .{ message, s });
    @breakpoint();
    rp2xxx.rom.reboot_to_bootsel_now() catch {};
    while (true) {}
}

fn core1() void {
    // 1. Setup allocator for protocol handler
    var buff: [512]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buff);
    const allocator = fba.allocator();

    while (true) {
        // Wait for usb data length in the fifo
        handleProcessRx(allocator);
        //handleInterrupts();
        //defer arena.deinit();
    }
}

fn handleProcessRx(allocator: std.mem.Allocator) void {
    if (rp2xxx.multicore.fifo.read()) |message_len| {

        // Get the slice into a reader
        shared_data_spinlock.lock();
        const incoming_data = shared_usb_rx_buff[0..message_len];
        shared_data_spinlock.unlock();
        var reader: std.Io.Reader = std.Io.Reader.fixed(incoming_data);

        // Get the response kind
        protocol_handler.handle_incoming_usb(allocator, &reader, &usb_writer) catch |err| {
            var err_buff: [64]u8 = undefined;
            const formatted_err = std.fmt.bufPrint(err_buff[0..], "Handle Err: {any}", .{err}) catch "format error";
            protocol_handler.encode_message(&usb_writer, allocator, .{ .error_response = .{ .message = formatted_err } }) catch {};
        };

        // Try the custom usb handler if no data is in the usb_writer
        if (usb_writer.buffered().len == 0) {
            if (custom_usb_handler) |handler| {
                var reader2 = std.Io.Reader.fixed(incoming_data);
                handler(allocator, &reader2, &usb_writer);
            }
        }

        // Copy it over to the shared tx buffer
        usbTxWrite(usb_writer.buffered());
        usb_writer.end = 0;
    }
}

pub fn usbTxWrite(buff: []const u8) void {
    if (buff.len == 0) return;
    shared_data_spinlock.lock();
    defer shared_data_spinlock.unlock();

    // Send the response
    std.mem.copyForwards(u8, &shared_usb_tx_buff, buff);
    rp2xxx.multicore.fifo.write_blocking(@intCast(buff.len));
}

var stack: [8192]u32 = undefined;

fn gpio_interrupt() callconv(.c) void {
    const cs = microzig.interrupt.enter_critical_section();
    defer cs.leave();

    for (interrupts_list[0..intr_index]) |i| {
        i.checkAndClear();
    }
}

var interrupts_list: [8]*Interrupt.Interrupt = undefined;
var intr_index: u8 = 0;

pub fn addInterrupt(intr: *Interrupt.Interrupt) void {
    interrupts_list[intr_index] = intr;
    intr.setup();
    intr_index += 1;
}

// Sets up the interrupts
fn enableInterrupts() void {
    microzig.interrupt.enable(.IO_IRQ_BANK0);
    microzig.interrupt.enable_interrupts();
    //_ = microzig.interrupt.set_handler(.IO_IRQ_BANK0, .{ .c = gpio_interrupt });
}

pub fn run() void {
    // Enable Interrupts
    //enableInterrupts();

    // Initialize the USB device
    const usb_dev = usb.Usb(.{});
    usb_dev.init_clk();
    usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

    // Start the 2nd core
    rp2xxx.multicore.launch_core1_with_stack(&core1, &stack);

    // Run the main loop
    // Main loop
    while (true) {
        // Poll for USB events
        usb_dev.task(false) catch unreachable;

        handleUsbRx();
        handleUsbTx();
    }
}

fn handleInterrupts() void {
    for (interrupts_list) |i| {
        if (i.triggered) {
            i.handler();
            i.triggered = false;
        }
    }
}

fn handleUsbRx() void {
    // Read in any USB data if there is any
    const rx_data = usb_read();
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
    if (rp2xxx.multicore.fifo.read()) |len| {
        shared_data_spinlock.lock();
        usb_write(shared_usb_tx_buff[0..len]);
        shared_data_spinlock.unlock();
    }
}

fn usb_read() []const u8 {
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

fn usb_write(buff: []const u8) void {
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
