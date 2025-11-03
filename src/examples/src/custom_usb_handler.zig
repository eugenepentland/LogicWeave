const logicweave = @import("logicweave");
const std = @import("std");
const microzig = @import("microzig");
const messages = @import("lw_core");
const rp2xxx = microzig.hal;
const has_rp2350b = rp2xxx.compatibility.has_rp2350b;
const lw = logicweave.init(messages);

fn usb_handler(allocator: std.mem.Allocator, message: messages.AppMessage, writer: *std.Io.Writer) void {
    if (message.kind) |kind_enum| {
        switch (kind_enum) {
            .read_voltage_request => {
                //const vout = read_voltmeter();
                const response = messages.AppMessage{ .kind = .{ .read_voltage_response = .{ .voltage = 100 } } };
                return response.encode(writer, allocator) catch {};
            },
            else => {
                const response = messages.AppMessage{ .kind = .{ .error_response = .{ .message = "Erorr reading request" } } };
                response.encode(writer, allocator) catch {};
            },
        }
    }
}

const voltmeter_switch = rp2xxx.gpio.num(28);
const resistancemeter_switch = rp2xxx.gpio.num(27);
const voltmeter_adc: rp2xxx.adc.Input = .ain6;
const resistance_adc: rp2xxx.adc.Input = .ain7;
const resistor_mux_a0 = rp2xxx.gpio.num(30);
const resistor_mux_a1 = rp2xxx.gpio.num(31);
const scale_factor: f32 = (56_000 + 10_000) / 10_000;

pub fn main() !void {
    setup();

    lw.custom_usb_handler = &usb_handler;
    lw.run();
}

fn setup() void {
    inline for (&.{ voltmeter_switch, resistancemeter_switch, resistor_mux_a0, resistor_mux_a1 }) |pin| {
        pin.set_function(.sio);
        pin.set_direction(.out);
    }

    inline for (&.{ voltmeter_adc, resistance_adc }) |pin| {
        rp2xxx.adc.configure_gpio_pin_num(pin);
        rp2xxx.adc.apply(.{});
    }
}

fn disable_all_circuits() void {
    voltmeter_switch.put(0);
    resistancemeter_switch.put(0);
}

fn enable_voltmeter() void {
    disable_all_circuits();
    voltmeter_switch.put(1);
    rp2xxx.time.sleep_ms(5);
}

fn enable_resistancemeter() void {
    disable_all_circuits();
    resistancemeter_switch.put(1);
    rp2xxx.time.sleep_ms(5);
}

fn read_voltmeter() f32 {
    enable_voltmeter();
    // Select the ADC
    rp2xxx.adc.select_input(voltmeter_adc);
    const sample = rp2xxx.adc.convert_one_shot_blocking(voltmeter_adc) catch 1;
    const vout: f32 = (@as(f32, @floatFromInt(sample)) / 4095) * 3.3;

    const v_in = vout * scale_factor * 1.01;
    disable_all_circuits();
    return v_in;
}
