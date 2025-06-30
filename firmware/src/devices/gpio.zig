const std = @import("std");
const microzig = @import("microzig");
const rp2040 = microzig.hal;
const time = rp2040.time;
const LedStrip = @import("./ws2812_led.zig");

const gpio = @This();

pin: rp2040.gpio.Pin,
pwm: rp2040.pwm.Pwm,
led_color: LedStrip.Color,
previous_led_color: LedStrip.Color,
level: u16 = 2000,
feeder_inserted: u1,
button_pressed: u1,
function: PinFunction,

const PinFunction = enum {
    pwm,
    gpio_output,
    gpio_input,
};

pub fn set_led_color(_: LedStrip, _: LedStrip.Color) void {
    return;
}

pub fn set_function(self: *@This(), function: PinFunction) void {
    if (function == self.function) return;

    switch (function) {
        .pwm => self.pin.set_function(.pwm),
        .gpio_input => {
            self.pin.set_function(.sio);
            self.pin.set_direction(.in);
        },
        .gpio_output => {
            self.pin.set_function(.sio);
            self.pin.set_direction(.in);
        },
    }
    self.function = function;
}

pub fn init(gpio_pin_num: u5, slice: u32, channel: rp2040.pwm.Channel) gpio {
    const pin = rp2040.gpio.num(gpio_pin_num);
    // Make sure its set to PWM
    pin.set_function(.pwm);

    const servo = gpio{
        .pwm = rp2040.pwm.Pwm{
            .slice_number = slice,
            .channel = channel,
        },
        .led_color = LedStrip.Color{ .r = 0, .g = 0, .b = 0 },
        .previous_led_color = LedStrip.Color{ .r = 0, .g = 0, .b = 0 },
        .pin = pin,
        .feeder_inserted = 0,
        .button_pressed = 0,
        .function = .pwm,
    };
    // Assumes 125 Mhz clock speed
    servo.pwm.slice().set_clk_div(250, 0);
    servo.pwm.slice().set_wrap(10000);
    return servo;
}

pub fn set_level(self: @This(), level: u16) u16 {
    // Set the level
    self.pwm.set_level(level);

    // Enable the PWM output
    self.pwm.slice().enable();

    return 200;
}

fn get_sleep_time_ms(self: @This(), level: u16) u16 {
    //const level: u16 = (54 * angle) / 10 + 250;
    var sleep_ms: u16 = 0;
    // Larger the multiplier, the slower it runs
    const multiplier: u16 = 30;

    if (level >= self.level) {
        sleep_ms = (level - self.level) * multiplier / 10;
    } else {
        sleep_ms = (self.level - level) * multiplier / 10;
    }

    return sleep_ms;
}
