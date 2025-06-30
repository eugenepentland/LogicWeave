const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const gpio = rp2xxx.gpio;
const Pio = rp2xxx.pio.Pio;
const StateMachine = rp2xxx.pio.StateMachine;

pub const LedStrip = @This();

pub const num_leds: usize = 16;

pub const Color = packed struct {
    _padding: u8 = 0,
    b: u8,
    g: u8,
    r: u8,
};

pio: Pio,
sm: StateMachine,
pin: gpio.Pin,
led_states: [num_leds]Color,

const ws2812_program = blk: {
    @setEvalBranchQuota(5000);
    break :blk rp2xxx.pio.assemble(
        \\;
        \\; Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
        \\;
        \\; SPDX-License-Identifier: BSD-3-Clause
        \\;
        \\.program ws2812
        \\.side_set 1
        \\
        \\.define public T1 2
        \\.define public T2 5
        \\.define public T3 3
        \\
        \\.wrap_target
        \\bitloop:
        \\    out x, 1       side 0 [T3 - 1] ; Side-set still takes place when instruction stalls
        \\    jmp !x do_zero side 1 [T1 - 1] ; Branch on the bit we shifted out. Positive pulse
        \\do_one:
        \\    jmp  bitloop   side 1 [T2 - 1] ; Continue driving high, for a long pulse
        \\do_zero:
        \\    nop            side 0 [T2 - 1] ; Or drive low, for a short pulse
        \\.wrap
    , .{}).get_program_by_name("ws2812");
};

pub fn init(
    pio_num: u2,
    sm_num: u8,
    gpio_pin_num: u5,
) !LedStrip {
    const pio_instance = rp2xxx.pio.num(pio_num);
    const sm_instance = @as(StateMachine, @enumFromInt(sm_num));
    const led_pin_instance = gpio.num(gpio_pin_num);

    pio_instance.gpio_init(led_pin_instance);
    pio_instance.sm_set_pindir(sm_instance, gpio_pin_num, 1, .out);

    const cycles_per_bit: comptime_int = ws2812_program.defines[0].value + //T1
        ws2812_program.defines[1].value + //T2
        ws2812_program.defines[2].value; //T3
    const div = @as(f32, @floatFromInt(rp2xxx.clock_config.sys.?.frequency())) /
        (800_000 * cycles_per_bit);

    try pio_instance.sm_load_and_start_program(sm_instance, ws2812_program, .{
        .clkdiv = rp2xxx.pio.ClkDivOptions.from_float(div),
        .pin_mappings = .{
            .side_set = .{
                .base = gpio_pin_num,
                .count = 1,
            },
        },
        .shift = .{
            .out_shiftdir = .left,
            .autopull = true,
            .pull_threshold = 24,
            .join_tx = true,
        },
    });
    pio_instance.sm_set_enabled(sm_instance, true);
    const default_color = Color{ .r = 0, .g = 0, .b = 0 };
    return LedStrip{
        .pio = pio_instance,
        .sm = sm_instance,
        .pin = led_pin_instance,
        .led_states = [_]Color{default_color} ** LedStrip.num_leds,
    };
}

// Function to set the color of a specific LED
pub fn setLed(self: *LedStrip, index: usize, color: Color) void {
    if (index < LedStrip.num_leds) {
        self.led_states[index] = color;
        self.updateLeds();
    }
}

// Function to set the color of a specific LED
pub fn setLedState(self: *LedStrip, index: usize, color: Color) void {
    if (index < LedStrip.num_leds) {
        self.led_states[index] = color;
    }
}

// Function to send the current LED states to the LEDs
pub fn updateLeds(self: *LedStrip) void {
    for (self.led_states) |color| {
        self.pio.sm_blocking_write(self.sm, @as(u32, @bitCast(color)));
    }
}
