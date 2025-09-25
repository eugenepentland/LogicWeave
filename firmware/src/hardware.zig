// src/hardware.zig
const std = @import("std");
const microzig = @import("microzig");
const Screen = @import("devices/E2206KS0E1.zig");
const PPS = @import("devices/ap33772s.zig");
const Graphics = @import("graphics.zig");
const definitions = @import("proto_gen/all.pb.zig"); // For BankVoltage enum
const fatfs = @import("zfat");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const peripherals = microzig.chip.peripherals;
const interrupt = microzig.hal.irq;

// --- Public Device Instances ---
pub var g: Graphics.Graphics = undefined;
pub var pps1: PPS = undefined;
pub var pps2: PPS = undefined;
pub var screen: Screen = undefined;
pub var global_fs: fatfs.FileSystem = undefined;
pub var log_commands: bool = false;
pub var log_filename_buffer: [32]u8 = undefined;
pub var log_filename_slice: []u8 = log_filename_buffer[0..];

// --- Constants ---
const N_PIXELS = 128;
const M_PIXELS = 248;
const TOTAL_BYTES_PER_FRAME = (N_PIXELS * M_PIXELS) / 8;
var last_trigger_time: u64 = 0;
const DEBOUCE_TIME_US: u64 = 25e4;
pub var pps1_btn_pressed = false;
pub var pps2_btn_pressed = false;

const pps1_ctrl_btn = rp2xxx.gpio.num(23);
const pps2_ctrl_btn = rp2xxx.gpio.num(24);

fn interruptIsReady() bool {
    const current_time = time.get_time_since_boot().to_us();
    if ((current_time - last_trigger_time) < DEBOUCE_TIME_US) return false;
    last_trigger_time = current_time;
    return true;
}

fn checkAndClearInterrupt(
    comptime io_bank_name: []const u8,
    comptime intr_reg_name: []const u8,
    comptime field_name: []const u8,
) bool {
    const io_bank = @field(peripherals, io_bank_name);
    var intr_reg = @field(io_bank, intr_reg_name);

    // 3. Read the current status of the interrupt register.
    const intr_status = intr_reg.read();

    // 4. Check the specific interrupt flag (e.g., GPIO23_EDGE_LOW) using its name.
    if (@field(intr_status, field_name) == 1) {
        // The interrupt is active. Now we clear it.
        intr_reg.modify_one(field_name, 1);

        return interruptIsReady();
    }

    return false;
}

// --- Interrupt Handler ---
fn gpio_interrupt() callconv(.c) void {
    const cs = microzig.interrupt.enter_critical_section();
    defer cs.leave();

    // Button to toggle CH1 PD power
    if (checkAndClearInterrupt("IO_BANK0", "INTR2", "GPIO23_EDGE_LOW")) {
        pps1.toggle();
        return;
    }

    // Button to toggle CH2 PD power
    if (checkAndClearInterrupt("IO_BANK0", "INTR3", "GPIO24_EDGE_LOW")) {
        pps2.toggle();
        return;
    }

    // CH1 PD interrupt
    if (checkAndClearInterrupt("IO_BANK0", "INTR0", "GPIO4_EDGE_LOW")) {
        pps1.enable(false);
        pps1.configureProtections(PPS.DEFAULT_CONFIG) catch {};
        return;
    }

    // CH2 PD interrupt
    if (checkAndClearInterrupt("IO_BANK0", "INTR0", "GPIO5_EDGE_LOW")) {
        pps2.enable(false);
        pps2.configureProtections(PPS.DEFAULT_CONFIG) catch {};
        return;
    }
}

fn setup_io_interrupts(
    pin: rp2xxx.gpio.Pin,
    comptime io_bank_name: []const u8,
    comptime intr_reg_name: []const u8,
    comptime field_name: []const u8,
) void {
    // Setup the GPIO pin as input with schmitt trigger
    pin.set_function(.sio);
    pin.set_direction(.in);
    pin.set_pull(.up);
    pin.set_schmitt_trigger(.enabled);

    // Enable the interrupt
    const io_bank = @field(peripherals, io_bank_name);
    var intr_reg = @field(io_bank, intr_reg_name);
    intr_reg.modify_one(field_name, 1);
}

fn init_gpio_bank_voltage() !void {
    inline for (&.{ 21, 22, 19, 20, 14, 15 }) |pin| {
        const gp = rp2xxx.gpio.num(pin);
        gp.set_function(.sio);
        gp.set_direction(.out);
    }
    try set_bank_voltage(1, .V3P3);
    try set_bank_voltage(2, .V3P3);
    try set_bank_voltage(3, .V3P3);
}

pub fn pps_init() !void {
    pps1 = try PPS.init(0, 1, 6, rp2xxx.i2c.instance.num(0));
    pps2 = try PPS.init(2, 3, 7, rp2xxx.i2c.instance.num(1));
}

pub fn screen_init() !void {
    screen = try Screen.init(
        rp2xxx.gpio.num(17),
        rp2xxx.gpio.num(18),
        rp2xxx.gpio.num(16),
        rp2xxx.gpio.num(25),
        rp2xxx.gpio.num(10),
        rp2xxx.gpio.num(11),
        rp2xxx.spi.instance.num(1),
    );
    try screen.cog_initial();
    g = Graphics.Graphics.init();
    try screen.global_update(&[_]u8{0x00} ** TOTAL_BYTES_PER_FRAME, &[_]u8{0x00} ** TOTAL_BYTES_PER_FRAME, .Normal, 0x19);
    g.refreshFrameBuffer();
}

pub fn update_screen() !void {
    const rotated_new_slice = Graphics.Graphics.getRotatedBuffer(g.frame_buffer);
    const rotated_old_slice = Graphics.Graphics.getRotatedBuffer(g.old_frame_buffer);
    try screen.global_update(&rotated_old_slice, &rotated_new_slice, .Fast, 0x19);
    g.refreshFrameBuffer();
}

// A single public init function to be called from main
pub fn init(_: std.mem.Allocator) !void {
    const pd1_int = rp2xxx.gpio.num(4);
    const pd2_int = rp2xxx.gpio.num(5);

    setup_io_interrupts(pps1_ctrl_btn, "IO_BANK0", "PROC0_INTE2", "GPIO23_EDGE_LOW");
    setup_io_interrupts(pps2_ctrl_btn, "IO_BANK0", "PROC0_INTE3", "GPIO24_EDGE_LOW");

    setup_io_interrupts(pd1_int, "IO_BANK0", "PROC0_INTE0", "GPIO4_EDGE_LOW");
    setup_io_interrupts(pd2_int, "IO_BANK0", "PROC0_INTE0", "GPIO5_EDGE_LOW");

    try init_gpio_bank_voltage();
    pps_init() catch {};
    try screen_init();

    interrupt.enable(.IO_IRQ_BANK0);
    interrupt.globally_enable();
    _ = interrupt.set_handler(.IO_IRQ_BANK0, .{ .c = gpio_interrupt });
}

// --- Public Hardware Control Functions ---
pub fn getGPIO(gpio_enum: u32) gpio.Pin {
    const gpio_num: u6 = @intCast(gpio_enum);
    return gpio.num(gpio_num);
}

pub fn getPPS(channel: u32) !*PPS {
    const pps = switch (channel) {
        1 => &pps1,
        2 => &pps2,
        else => return error.InvalidChannel,
    };
    return pps;
}

pub fn set_bank_voltage(bank: u32, voltage: definitions.BankVoltage) !void {
    var gpio_in1: gpio.Pin = undefined;
    var gpio_in2: gpio.Pin = undefined;
    switch (bank) {
        3 => {
            gpio_in1 = rp2xxx.gpio.num(19);
            gpio_in2 = rp2xxx.gpio.num(20);
        },
        2 => {
            gpio_in1 = rp2xxx.gpio.num(21);
            gpio_in2 = rp2xxx.gpio.num(22);
        },
        1 => {
            gpio_in1 = rp2xxx.gpio.num(15);
            gpio_in2 = rp2xxx.gpio.num(14);
        },
        else => return error.InvalidIOBank,
    }
    switch (voltage) {
        .V1P8 => {
            gpio_in1.put(1);
            gpio_in2.put(1);
        },
        .V3P3 => {
            gpio_in1.put(0);
            gpio_in2.put(1);
        },
        .V5P0 => {
            gpio_in1.put(1);
            gpio_in2.put(0);
        },
        else => return error.InvalidBankVoltage,
    }
}

pub fn poll_and_update_display() !void {
    g.clear(.White);

    try poll_voltage_current(&pps2, 5);
    try poll_voltage_current(&pps1, 150);

    try screen.global_update(Graphics.Graphics.getRotatedBuffer(g.old_frame_buffer)[0..], Graphics.Graphics.getRotatedBuffer(g.frame_buffer)[0..], .Fast, 25);
    g.refreshFrameBuffer();
}

fn poll_voltage_current(pps: *PPS, x_offset: u8) !void {
    const voltage_req = pps.readRequestedVoltageMv() catch 0;
    const current_req = pps.readRequestedCurrentMa() catch 0;
    const voltage = pps.readVoltageMv() catch 0;
    const current = pps.readCurrentMa() catch 0;
    var buff: [10]u8 = undefined;
    var msg: []const u8 = undefined;
    msg = try std.fmt.bufPrint(&buff, "S {d}.{d:02}V", .{ voltage_req / 1000, (voltage_req % 1000) / 10 });
    try g.drawString(msg, x_offset, 5);
    msg = try std.fmt.bufPrint(&buff, "M {d}.{d:02}V", .{ voltage / 1000, (voltage % 1000) / 10 });
    try g.drawString(msg, x_offset, 35);
    msg = try std.fmt.bufPrint(&buff, "S {d}.{d:02}A", .{ current_req / 1000, (current_req % 1000) / 10 });
    try g.drawString(msg, x_offset, 75);
    msg = try std.fmt.bufPrint(&buff, "M {d}.{d:02}A", .{ current / 1000, (current % 1000) / 10 });
    try g.drawString(msg, x_offset, 100);
}
