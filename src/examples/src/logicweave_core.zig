const logicweave = @import("logicweave");
const std = @import("std");
const microzig = @import("microzig");
const messages = @import("lw_core");
const rp2xxx = microzig.hal;
const has_rp2350b = rp2xxx.compatibility.has_rp2350b;
const lw = logicweave.init(messages);

fn usb_handler(_: std.mem.Allocator, message: messages.RequestMessage) messages.ResponseMessage.kind_union {
    if (message.kind) |kind_enum| {
        switch (kind_enum) {
            .read_voltage_request => {
                const vout = read_voltmeter();
                return .{ .read_voltage_response = .{ .voltage = vout } };
            },
            .read_resistance_request => {
                const rx = read_resistancemeter();
                return .{ .read_resistance_response = .{ .resistance = rx } };
            },
            else => {},
        }
    }
    return .{ .error_response = .{ .message = "Erorr reading custom request" } };
}

const voltmeter_switch = rp2xxx.gpio.num(28);
const resistancemeter_switch = rp2xxx.gpio.num(27);
const voltmeter_adc: rp2xxx.adc.Input = .ain6;
const resistance_adc: rp2xxx.adc.Input = .ain7;
const resistor_mux_a0 = rp2xxx.gpio.num(30);
const resistor_mux_a1 = rp2xxx.gpio.num(31);
const scale_factor: f32 = (56_000.0 + 10_000.0) / 10_000.0;

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
    rp2xxx.time.sleep_ms(10);
}

fn enable_resistancemeter() void {
    disable_all_circuits();
    resistancemeter_switch.put(1);
    rp2xxx.time.sleep_ms(10);
}

fn read_voltmeter() f32 {
    enable_voltmeter();
    rp2xxx.adc.select_input(voltmeter_adc);
    const sample = rp2xxx.adc.convert_one_shot_blocking(voltmeter_adc) catch 1;
    const vout: f32 = (@as(f32, @floatFromInt(sample)) / 4095.0) * 3.3;

    const v_in = vout * scale_factor * 1.01;
    disable_all_circuits();
    return v_in;
}

const ResistorBank = enum {
    R_100_OHM,
    R_1K_OHM,
    R_10K_OHM,
    R_100K_OHM,
};

const target_sample: u12 = 2048;

fn read_resistancemeter() f32 {
    // 1. Enable the resistance circuit
    enable_resistancemeter();
    rp2xxx.adc.select_input(resistance_adc);

    // Initial values for sweep comparison
    var best_sample: u12 = 0;
    var best_sample_diff: u12 = 4095; // Max possible difference
    var best_bank: ResistorBank = .R_100K_OHM; // Will be overwritten

    const banks = std.enums.values(ResistorBank);

    // 2. Sweep all banks to find the best one
    for (banks) |bank| {
        const measured_sample = measure_bank_sample(bank);

        // Calculate the absolute difference from the target sample
        const value_diff = if (measured_sample > target_sample)
            measured_sample - target_sample
        else
            target_sample - measured_sample;

        // Update the best bank if this one is closer to the target
        if (best_sample_diff > value_diff) {
            best_sample_diff = value_diff;
            best_sample = measured_sample;
            best_bank = bank;
        }
    }

    // 3. Calculate the final resistance using the best bank and its measured sample
    const calculated_resistance = calculate_measured_resistance(best_bank, best_sample);

    // 4. Disable the circuit
    disable_all_circuits();

    return calculated_resistance;
}

const vin: f32 = 3.3; // Defined at the file level

fn calculate_measured_resistance(bank: ResistorBank, sample: u12) f32 {
    // 1. Determine R_bank (R_pullup_known)
    const r_bank: f32 = switch (bank) {
        .R_100_OHM => 100.0,
        .R_1K_OHM => 1000.0,
        .R_10K_OHM => 10000.0,
        .R_100K_OHM => 100000.0,
    };

    // 2. Convert ADC sample (u12) to V_ADC (f32)
    const v_adc: f32 = (@as(f32, @floatFromInt(sample)) / 4095.0) * vin;

    // Check for short circuit (Rx = 0) - V_ADC is close to 0
    if (v_adc < 0.01) {
        return 0.0;
    }

    // 3. Calculate resistance Rx = R_bank * V_ADC / (Vin - V_ADC)
    const voltage_drop_r_bank = vin - v_adc;

    // Use the formula Rx = R_bank * (V_ADC / V_Rbank_Drop)
    const r_x = r_bank * (v_adc / voltage_drop_r_bank);

    return r_x;
}

fn measure_bank_sample(bank: ResistorBank) u12 {
    switch (bank) {
        .R_100_OHM => {
            resistor_mux_a1.put(0);
            resistor_mux_a0.put(0);
        },
        .R_1K_OHM => {
            resistor_mux_a1.put(0);
            resistor_mux_a0.put(1);
        },
        .R_10K_OHM => {
            resistor_mux_a1.put(1);
            resistor_mux_a0.put(0);
        },
        .R_100K_OHM => {
            resistor_mux_a1.put(1);
            resistor_mux_a0.put(1);
        },
    }
    rp2xxx.time.sleep_ms(5);
    return rp2xxx.adc.convert_one_shot_blocking(resistance_adc) catch 1;
}
