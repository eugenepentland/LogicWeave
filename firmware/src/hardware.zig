// src/hardware.zig
const std = @import("std");
const microzig = @import("microzig");
const Screen = @import("devices/E2206KS0E1.zig");
const PPS = @import("devices/ap33772s.zig");
const Graphics = @import("graphics.zig");
const definitions = @import("proto_gen/all.pb.zig"); // For BankVoltage enum
const SD = @import("devices/sd.zig");
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
pub var sd: SD.SD_Driver = undefined;
pub var sd_disk: SD.SdCardDisk = undefined;
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

// --- Interrupt Handler ---
fn gpio_interrupt() callconv(.c) void {
    const cs = microzig.interrupt.enter_critical_section();
    defer cs.leave();

    const intr2_status = peripherals.IO_BANK0.INTR2.read();
    if (intr2_status.GPIO23_EDGE_LOW == 1) {
        defer peripherals.IO_BANK0.INTR2.modify(.{ .GPIO23_EDGE_LOW = 1 });
        const current_time = time.get_time_since_boot().to_us();
        if ((current_time - last_trigger_time) > DEBOUCE_TIME_US) {
            last_trigger_time = current_time;
            pps1.toggle();
        }
    }

    const intr3_status = peripherals.IO_BANK0.INTR3.read();
    if (intr3_status.GPIO24_EDGE_HIGH == 1) {
        defer peripherals.IO_BANK0.INTR3.modify(.{ .GPIO24_EDGE_HIGH = 1 });
        const current_time = time.get_time_since_boot().to_us();
        if ((current_time - last_trigger_time) > DEBOUCE_TIME_US) {
            last_trigger_time = current_time;
            pps2.toggle();
        }
    }

    const int_status = peripherals.IO_BANK0.INTR0.read();
    if (int_status.GPIO4_EDGE_LOW == 1) {
        defer peripherals.IO_BANK0.INTR0.modify(.{ .GPIO4_EDGE_LOW = 1 });
        pps1.enable(false);
        pps1.configureProtections(PPS.DEFAULT_CONFIG) catch {};
    }
    if (int_status.GPIO5_EDGE_LOW == 1) {
        defer peripherals.IO_BANK0.INTR0.modify(.{ .GPIO5_EDGE_LOW = 1 });
        pps2.enable(false);
        pps2.configureProtections(PPS.DEFAULT_CONFIG) catch {};
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
}

fn init_pd_interrupts() void {
    const pd1_int = rp2xxx.gpio.num(4);
    const pd2_int = rp2xxx.gpio.num(5);

    pd1_int.set_function(.sio);
    pd1_int.set_direction(.in);
    pd1_int.set_schmitt_trigger(.enabled);

    pd2_int.set_function(.sio);
    pd2_int.set_direction(.in);
    pd2_int.set_schmitt_trigger(.enabled);

    peripherals.IO_BANK0.PROC0_INTE0.modify(.{ .GPIO4_EDGE_LOW = 1 });
    peripherals.IO_BANK0.PROC0_INTE0.modify(.{ .GPIO5_EDGE_LOW = 1 });
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

fn sd_init(alloc: std.mem.Allocator) !void {
    sd = try SD.SD_Driver.init(
        alloc,
        rp2xxx.gpio.num(9),
        rp2xxx.gpio.num(10),
        rp2xxx.gpio.num(11),
        rp2xxx.gpio.num(8),
        rp2xxx.spi.instance.num(1),
        100_000,
    );
}

// A single public init function to be called from main
pub fn init(alloc: std.mem.Allocator) !void {
    init_pd_interrupts();
    init_pwr_buttons();
    try init_gpio_bank_voltage();
    pps_init() catch {};
    try screen_init();
    try sd_init(alloc);

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
