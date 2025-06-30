// src/hardware.zig
const std = @import("std");
const microzig = @import("microzig");
const Screen = @import("devices/E2206KS0E1.zig");
const PPS = @import("devices/ap33772s.zig");
const Graphics = @import("graphics.zig");
const definitions = @import("proto_gen/all.pb.zig"); // For BankVoltage enum

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

// --- Constants ---
const N_PIXELS = 128;
const M_PIXELS = 248;
const TOTAL_BYTES_PER_FRAME = (N_PIXELS * M_PIXELS) / 8;

const pps1_ctrl_btn = rp2xxx.gpio.num(23);
const pps2_ctrl_btn = rp2xxx.gpio.num(24);

// --- Interrupt Handler ---
fn gpio_interrupt() callconv(.c) void {
    const cs = microzig.interrupt.enter_critical_section();
    defer cs.leave();

    const intr2_status = peripherals.IO_BANK0.INTR2.read();
    if (intr2_status.GPIO23_EDGE_LOW == 1) {
        pps1.toggle();
        peripherals.IO_BANK0.INTR2.modify(.{ .GPIO23_EDGE_LOW = 1 });
    }

    const intr3_status = peripherals.IO_BANK0.INTR3.read();
    if (intr3_status.GPIO24_EDGE_LOW == 1) {
        pps2.toggle();
        peripherals.IO_BANK0.INTR3.modify(.{ .GPIO24_EDGE_LOW = 1 });
    }
}

// --- Initialization Functions ---
fn init_pwr_buttons() void {
    pps1_ctrl_btn.set_function(.sio);
    pps1_ctrl_btn.set_direction(.in);
    pps1_ctrl_btn.set_pull(.up);
    pps1_ctrl_btn.set_schmitt_trigger(.enabled);

    pps2_ctrl_btn.set_function(.sio);
    pps2_ctrl_btn.set_direction(.in);
    pps2_ctrl_btn.set_pull(.up);
    pps2_ctrl_btn.set_schmitt_trigger(.enabled);

    peripherals.IO_BANK0.PROC0_INTE2.modify(.{ .GPIO23_EDGE_LOW = 1 });
    peripherals.IO_BANK0.PROC0_INTE3.modify(.{ .GPIO24_EDGE_LOW = 1 });
    interrupt.enable(.IO_IRQ_BANK0);
    interrupt.globally_enable();
    _ = interrupt.set_handler(.IO_IRQ_BANK0, .{ .c = gpio_interrupt });
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

fn pps_init() !void {
    pps1 = try PPS.init(0, 1, 6, rp2xxx.i2c.instance.num(0));
    pps2 = try PPS.init(2, 3, 7, rp2xxx.i2c.instance.num(1));
}

fn screen_init() !void {
    screen = try Screen.init(
        rp2xxx.gpio.num(17),
        rp2xxx.gpio.num(18),
        rp2xxx.gpio.num(16),
        rp2xxx.gpio.num(25),
        rp2xxx.gpio.num(10),
        rp2xxx.gpio.num(11),
        rp2xxx.spi.instance.num(1),
    );
    screen.cog_initial();
    g = Graphics.Graphics.init();
    screen.global_update(&[_]u8{0x00} ** TOTAL_BYTES_PER_FRAME, &[_]u8{0x00} ** TOTAL_BYTES_PER_FRAME, .Normal, 0x19);
    g.refreshFrameBuffer();
}

// A single public init function to be called from main
pub fn init() !void {
    init_pwr_buttons();
    try init_gpio_bank_voltage();
    try pps_init();
    try screen_init();
}

// --- Public Hardware Control Functions ---
pub fn getGPIO(gpio_enum: u32) gpio.Pin {
    const gpio_num: u6 = @intCast(gpio_enum);
    return gpio.num(gpio_num);
}

pub fn getPPS(channel: u32) !*PPS {
    return switch (channel) {
        1 => &pps1,
        2 => &pps2,
        else => error.InvalidChannel,
    };
}

pub fn set_bank_voltage(bank: u32, voltage: definitions.BankVoltage) !void {
    var gpio_in1: gpio.Pin = undefined;
    var gpio_in2: gpio.Pin = undefined;
    switch (bank) {
        1 => {
            gpio_in1 = rp2xxx.gpio.num(15);
            gpio_in2 = rp2xxx.gpio.num(14);
        },
        2 => {
            gpio_in1 = rp2xxx.gpio.num(19);
            gpio_in2 = rp2xxx.gpio.num(20);
        },
        3 => {
            gpio_in1 = rp2xxx.gpio.num(21);
            gpio_in2 = rp2xxx.gpio.num(22);
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

    screen.global_update(Graphics.Graphics.getRotatedBuffer(g.old_frame_buffer)[0..], Graphics.Graphics.getRotatedBuffer(g.frame_buffer)[0..], .Fast, 25);
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
