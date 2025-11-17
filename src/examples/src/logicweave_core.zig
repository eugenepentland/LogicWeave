const logicweave = @import("logicweave");
const std = @import("std");
const microzig = @import("microzig");
const messages = @import("lw_core");
const AP33772S = @import("drivers/AP33772S.zig");
const INA700 = @import("drivers/INA700.zig");
const MCP47FEB22 = @import("drivers/MCP47FEB22.zig");
const I2CMutex = @import("drivers/i2c_mutex.zig").I2CMutex;
const gpio = rp2xxx.gpio;
const rp2xxx = microzig.hal;
const has_rp2350b = rp2xxx.compatibility.has_rp2350b;
const lw = logicweave.init(messages);
var err_msg_buff: [48]u8 = undefined;

// used as event flag to keep IRQ handler fast
var event: ?gpio.IrqTrigger = null;

fn handle_err(msg: []const u8, err: anytype) messages.ResponseMessage.kind_union {
    const err_msg = std.fmt.bufPrint(&err_msg_buff, "{s}: {any}", .{ msg, err }) catch "err";
    return .{ .error_response = .{ .message = err_msg } };
}

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
            .read_pd_request => {
                const requested_voltage = usb_pd.readRequestedVoltageMv() catch 0;
                const requested_current = usb_pd.readRequestedCurrentMa() catch 0;
                const measured_voltage = usb_pd.readVoltageMv() catch 0;
                const measured_current = usb_pd.readCurrentMa() catch 0;

                return .{ .read_pd_response = .{
                    .requested_voltage_mv = requested_voltage,
                    .requested_current_ma = requested_current,
                    .measured_voltage_mv = measured_voltage,
                    .measured_current_ma = measured_current,
                } };
            },
            .set_psu_output_request => |request| {
                const is_enabled: u1 = @intFromBool(request.state);
                switch (request.channel) {
                    1 => {
                        ch1_en.put(is_enabled);
                        is_enabled_ch1 = request.state;
                    },
                    2 => {
                        ch2_en.put(is_enabled);
                        is_enabled_ch2 = request.state;
                    },
                    else => {},
                }

                return .{ .set_psu_output_response = .{ .status = 200 } };
            },
            .read_power_monitor_request => {
                var voltage_ch1 = pm_ch1.readVoltage() catch |err| return handle_err("pm1 v", err);
                rp2xxx.time.sleep_ms(1);
                var voltage_ch2 = pm_ch2.readVoltage() catch |err| return handle_err("pm2 v", err);
                rp2xxx.time.sleep_ms(1);
                var current_ch1 = pm_ch1.readCurrent() catch |err| return handle_err("pm1 c", err);
                rp2xxx.time.sleep_ms(1);
                var current_ch2 = pm_ch2.readCurrent() catch |err| return handle_err("pm2 c", err);
                if (!is_enabled_ch1) {
                    voltage_ch1 = 0.0;
                    current_ch1 = 0.0;
                }
                if (!is_enabled_ch2) {
                    voltage_ch2 = 0.0;
                    current_ch2 = 0.0;
                }
                return .{
                    .read_power_monitor_response = .{ .channel_1 = .{
                        .voltage = voltage_ch1,
                        .current = current_ch1,
                        .current_limit = current_limit_ch1,
                        .requested_voltage = requested_voltage_ch1,
                        .is_enabled = is_enabled_ch1,
                    }, .channel_2 = .{
                        .voltage = voltage_ch2,
                        .current = current_ch2,
                        .current_limit = current_limit_ch2,
                        .requested_voltage = requested_voltage_ch2,
                        .is_enabled = is_enabled_ch2,
                    } },
                };
            },
            .configure_psu_request => |request| {
                const channel: MCP47FEB22.Channel = @enumFromInt(request.channel - 1);
                const dac_value = voltage_to_dac(request.voltage);
                if (request.channel == 1) {
                    current_limit_ch1 = request.current_limit;
                    requested_voltage_ch1 = request.voltage;
                } else {
                    current_limit_ch2 = request.current_limit;
                    requested_voltage_ch2 = request.voltage;
                }
                dac.set_voltage(channel, dac_value) catch |err| return handle_err("DAC", err);
                return .{ .configure_psu_response = .{ .status = 200 } };
            },
            else => {},
        }
    }
    return .{ .error_response = .{ .message = "Erorr reading custom request" } };
}
const time = rp2xxx.time;
const voltmeter_switch = rp2xxx.gpio.num(28);
const resistancemeter_switch = rp2xxx.gpio.num(27);
const voltmeter_adc: rp2xxx.adc.Input = .ain6;
const resistance_adc: rp2xxx.adc.Input = .ain7;
const resistor_mux_a0 = rp2xxx.gpio.num(30);
const resistor_mux_a1 = rp2xxx.gpio.num(31);
const scale_factor: f32 = (56_000.0 + 10_000.0) / 10_000.0;
const sda = rp2xxx.gpio.num(0);
const ch1_en = rp2xxx.gpio.num(24);
const ch2_en = rp2xxx.gpio.num(25);
const pd_alert = gpio.num(2);
const scl = rp2xxx.gpio.num(1);
var ic2_instance = I2CMutex.init(rp2xxx.i2c.instance.num(0));
const uart_tx_pin = rp2xxx.gpio.num(12);
const uart = rp2xxx.uart.instance.num(0);
var usb_pd: AP33772S = undefined;
var pm_ch1: INA700 = undefined;
var pm_ch2: INA700 = undefined;
var is_enabled_ch1: bool = false;
var is_enabled_ch2: bool = false;
var current_limit_ch1: f32 = 1.0;
var current_limit_ch2: f32 = 1.0;
var requested_voltage_ch1: f32 = 3.3;
var requested_voltage_ch2: f32 = 3.3;
var max_voltage: f32 = 5.0;
var dac: MCP47FEB22 = undefined;
const logging = false;

fn callback_alt() linksection(".ram_text") callconv(.c) void {
    var iter = gpio.IrqEventIter{};
    while (iter.next()) |e| {
        event = e;
    }
}

pub const microzig_options = microzig.Options{
    .log_level = .debug,
    .logFn = rp2xxx.uart.log,
    //.interrupts = .{ .IO_IRQ_BANK0 = .{ .c = callback_alt } },
};

pub fn main() !void {
    setup();
    std.log.info("Starting!", .{});
    lw.custom_usb_handler = &usb_handler;
    lw.setup();
    const usb_dev = rp2xxx.usb.Usb(.{});
    var old: u64 = time.get_time_since_boot().to_us();
    var new: u64 = 0;

    while (true) {
        usb_dev.task(false) catch unreachable;
        lw.handleUsbRx();
        lw.handleUsbTx();

        // check for over current every 5 ms
        if (new - old > 50000) {
            check_current_limits();
            old = time.get_time_since_boot().to_us();
        }
        new = time.get_time_since_boot().to_us();
    }
}

fn check_current_limits() void {
    // read the current draw on the two channels
    const current_ch1 = pm_ch1.readCurrent() catch 5.0;
    const current_ch2 = pm_ch2.readCurrent() catch 5.0;
    if (current_ch1 > current_limit_ch1) {
        ch1_en.put(0);
        is_enabled_ch1 = false;
    }
    if (current_ch2 > current_limit_ch2) {
        ch2_en.put(0);
        is_enabled_ch2 = false;
    }
}

fn setup() void {
    // Setup the interrupts
    pd_alert.set_function(.sio);
    pd_alert.set_direction(.in);
    //pd_alert.set_irq_enabled(gpio.IrqEvents{ .fall = 1, .rise = 0 }, true);
    //microzig.interrupt.enable(.IO_IRQ_BANK0);

    // Set the output pins
    inline for (&.{ voltmeter_switch, resistancemeter_switch, resistor_mux_a0, resistor_mux_a1, ch2_en, ch1_en }) |pin| {
        pin.set_function(.sio);
        pin.set_direction(.out);
    }

    // Setup the ADC pins
    inline for (&.{ voltmeter_adc, resistance_adc }) |pin| {
        rp2xxx.adc.configure_gpio_pin_num(pin);
        rp2xxx.adc.apply(.{});
    }

    // Setup the I2C pins
    inline for (&.{ scl, sda }) |pin| {
        pin.set_slew_rate(.slow);
        pin.set_schmitt_trigger_enabled(true);
        pin.set_function(.i2c);
    }

    ic2_instance.apply(.{
        .clock_config = rp2xxx.clock_config,
    });

    if (logging) {
        uart_tx_pin.set_function(.uart);

        uart.apply(.{
            .clock_config = rp2xxx.clock_config,
        });
        rp2xxx.uart.init_logger(uart);
        std.log.info("Starting!", .{});
    }
    pm_ch1 = INA700.init(ic2_instance, @enumFromInt(0x46));
    pm_ch2 = INA700.init(ic2_instance, @enumFromInt(0x44));
    usb_pd = AP33772S.init(ic2_instance);
    dac = MCP47FEB22.init(ic2_instance);

    set_max_pd_voltage() catch |err| {
        std.log.debug("Error setting the pdo: {}", .{err});
    };
}

const V_OUT_MAX: f32 = 17.0;
const V_OUT_MIN: f32 = 0.7;
const DAC_MAX: u16 = 4095;
const DAC_MIN: u16 = 0;

fn voltage_to_dac(voltage: f32) u16 {
    const clamped_voltage: f32 = @max(V_OUT_MIN, @min(V_OUT_MAX, voltage));
    const voltage_range: f32 = V_OUT_MAX - V_OUT_MIN;
    const dac_range: f32 = @floatFromInt(DAC_MAX - DAC_MIN);
    const dac_value: u16 = @intFromFloat((V_OUT_MAX - clamped_voltage) * (@divFloor(dac_range, voltage_range)));
    return @max(DAC_MIN, @min(DAC_MAX, dac_value));
}

fn set_max_pd_voltage() !void {
    // Find what the max pdo voltage is
    var index: usize = 0;
    var pdo_max: AP33772S.SRC_PDO = undefined;
    for (1..14) |i| {
        const source_pdo = try usb_pd.readSourcePDO(@intCast(i));
        if (source_pdo.type == 1) continue;
        if (source_pdo.voltage_max > pdo_max.voltage_max) {
            pdo_max = source_pdo;
            index = i;
        }
    }

    // Set the to the max pdo
    requestPDOAndVerify(.{
        .current_select = pdo_max.current_max_code,
        .pdo_index = @intCast(index),
        .voltage_select = 0x00,
    }, 250) catch {
        // when there is no PD source connected
        pdo_max.voltage_max = 50;
        pdo_max.current_max_code = 4;
    };

    const max_voltage_mv_f32: f32 = @floatFromInt(pdo_max.get_voltage_mv(false));
    max_voltage = @divFloor(max_voltage_mv_f32, 1000);

    std.log.debug("Max output Voltage: {d}V", .{max_voltage});
}

fn requestPDOAndVerify(pdo_request: AP33772S.PDORequest, timeout_ms: u32) !void {
    try usb_pd.requestPDO(pdo_request);
    const start_time = rp2xxx.time.get_time_since_boot().to_us();
    var current_time = rp2xxx.time.get_time_since_boot().to_us();

    while (current_time - start_time < timeout_ms * 1000) {
        const result = try usb_pd.getPdMessageResult();
        if (result == 1) {
            std.log.info("Took {d}us to connect", .{rp2xxx.time.get_time_since_boot().to_us() - start_time});
            return;
        }
        if (result != 0) return error.PDORequestFailed;
        rp2xxx.time.sleep_ms(10);
        current_time = rp2xxx.time.get_time_since_boot().to_us();
    }
    return error.PDOTimeout;
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
