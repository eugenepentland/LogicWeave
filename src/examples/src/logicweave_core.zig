const std = @import("std");
const microzig = @import("microzig");
const logicweave = @import("logicweave");

// Drivers
const AP33772S = @import("drivers/AP33772S.zig");
const INA700 = @import("drivers/INA700.zig");
const MCP47FEB22 = @import("drivers/MCP47FEB22.zig");
const I2CMutex = @import("drivers/i2c_mutex.zig").I2CMutex;
const messages = @import("lw_core");

// HAL Shortcuts
const rp2xxx = microzig.hal;
const gpio = rp2xxx.gpio;
const time = rp2xxx.time;

// --------------------------------------------------------------------------
// Configuration & Constants
// --------------------------------------------------------------------------

const logging = false;
const flash_target_offset: u32 = 256 * 1024;
const flash_target_contents = @as([*]const u8, @ptrFromInt(rp2xxx.flash.XIP_BASE + flash_target_offset));

// PSU / DAC Constants
const V_OUT_MAX: f32 = 17.0;
const V_OUT_MIN: f32 = 0.7;
const DAC_MAX: u16 = 4095;
const DAC_MIN: u16 = 0;

// Resistance Meter Constants
const scale_factor: f32 = (56_000.0 + 10_000.0) / 10_000.0;
const vin: f32 = 3.3;
const target_sample: u12 = 2048;

// --------------------------------------------------------------------------
// Pin Definitions
// --------------------------------------------------------------------------

const pins = struct {
    // I2C
    const sda = gpio.num(0);
    const scl = gpio.num(1);

    // USB PD
    const pd_alert = gpio.num(2);

    // UART
    const uart_tx = gpio.num(12);

    // Power Supply Control
    const ch1_enable = gpio.num(24);
    const ch2_enable = gpio.num(25);

    // Analog / Metering Mux & Switches
    const resistancemeter_switch = gpio.num(27);
    const voltmeter_switch = gpio.num(28);
    const resistor_mux_a0 = gpio.num(30);
    const resistor_mux_a1 = gpio.num(31);

    // ADC Inputs
    const voltmeter_adc: rp2xxx.adc.Input = .ain6;
    const resistance_adc: rp2xxx.adc.Input = .ain7;
};

// --------------------------------------------------------------------------
// Global State & Drivers
// --------------------------------------------------------------------------

// LogicWeave & Buffers
const lw = logicweave.init(messages, .lw_core);
var err_msg_buff: [48]u8 = undefined;
var flash_buff: [rp2xxx.flash.PAGE_SIZE]u8 = undefined;
var event: ?gpio.IrqTrigger = null; // Used as event flag to keep IRQ handler fast

// Hardware Drivers
var ic2_instance = I2CMutex.init(rp2xxx.i2c.instance.num(0));
var uart = rp2xxx.uart.instance.num(0);
var usb_pd: AP33772S = undefined;
var pm_ch1: INA700 = undefined;
var pm_ch2: INA700 = undefined;
var dac: MCP47FEB22 = undefined;

// PSU State
var is_enabled_ch1: bool = false;
var is_enabled_ch2: bool = false;
var current_limit_ch1: f32 = 1.0;
var current_limit_ch2: f32 = 1.0;
var requested_voltage_ch1: f32 = 3.3;
var requested_voltage_ch2: f32 = 3.3;
var max_voltage: f32 = 5.0;

// --------------------------------------------------------------------------
// MicroZig Options
// --------------------------------------------------------------------------

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

// --------------------------------------------------------------------------
// Main Entry Point
// --------------------------------------------------------------------------

pub fn main() !void {
    setup();
    std.log.info("Starting!", .{});

    lw.custom_usb_handler = &usb_handler;
    lw.setup();

    const usb_dev = rp2xxx.usb.Usb(.{});
    usb_dev.init_clk();
    usb_dev.init_device(&lw.usb_config.DEVICE_CONFIGURATION) catch unreachable;
    var old_time: u64 = time.get_time_since_boot().to_us();
    var new_time: u64 = old_time;

    while (true) {
        usb_dev.task(false) catch unreachable;
        lw.handleUsbRx(.driver);
        lw.handleUsbRx(.webui);
        lw.handleUsbTx();

        defer new_time = time.get_time_since_boot().to_us();

        // Check for over-current every 50ms (50,000 us)
        if (new_time - old_time > 50_000) {
            //check_current_limits();
            old_time = time.get_time_since_boot().to_us();
        }
    }
}

fn setup() void {

    // 2. Setup Output Pins
    inline for (&.{ pins.voltmeter_switch, pins.resistancemeter_switch, pins.resistor_mux_a0, pins.resistor_mux_a1, pins.ch2_enable, pins.ch1_enable }) |pin| {
        pin.set_function(.sio);
        pin.set_direction(.out);
    }

    // 3. Setup ADC
    inline for (&.{ pins.voltmeter_adc, pins.resistance_adc }) |pin| {
        rp2xxx.adc.configure_gpio_pin_num(pin);
        rp2xxx.adc.apply(.{});
    }

    // 4. Setup I2C
    inline for (&.{ pins.scl, pins.sda }) |pin| {
        pin.set_slew_rate(.slow);
        pin.set_schmitt_trigger_enabled(true);
        pin.set_function(.i2c);
    }
    ic2_instance.apply(.{ .clock_config = rp2xxx.clock_config });

    // 5. Logging / UART
    if (logging) {
        pins.uart_tx.set_function(.uart);
        uart.apply(.{ .clock_config = rp2xxx.clock_config });
        rp2xxx.uart.init_logger(uart);
        std.log.info("Starting!", .{});
    }

    // 6. Init Drivers
    pm_ch1 = INA700.init(ic2_instance, @enumFromInt(0x46));
    pm_ch2 = INA700.init(ic2_instance, @enumFromInt(0x44));
    usb_pd = AP33772S.init(ic2_instance);
    dac = MCP47FEB22.init(ic2_instance);

    const dac0_val = voltage_to_dac(requested_voltage_ch1);
    const dac1_val = voltage_to_dac(requested_voltage_ch2);
    dac.set_voltage(.dac0, dac0_val) catch {};
    dac.set_voltage(.dac1, dac1_val) catch {};

    set_max_pd_voltage() catch |err| {
        std.log.debug("Error setting the pdo: {}", .{err});
    };
}

// --------------------------------------------------------------------------
// USB Handler & Logic
// --------------------------------------------------------------------------

fn usb_handler(_: std.mem.Allocator, message: messages.RequestMessage) messages.ResponseMessage.kind_union {
    if (message.kind) |kind_enum| {
        switch (kind_enum) {
            .read_calibration_data_request => {
                const ftc = @as([*]const u8, @ptrFromInt(rp2xxx.flash.XIP_BASE + flash_target_offset));
                const offset = std.mem.bytesToValue(f32, ftc[0..4]);
                return .{ .read_calibration_data_response = .{
                    .probe_zero_offset = offset,
                } };
            },
            .zero_probes_request => {
                //_ = read_resistancemeter(false);
                const result = read_resistancemeter(false, .R_100_OHM);
                if (result.resistance > 10) {
                    return .{ .error_response = .{ .message = "Make sure probes are touching, measured more than 10 ohms" } };
                }
                // wipe the page
                //rp2xxx.flash.range_erase(flash_target_offset, rp2xxx.flash.SECTOR_SIZE);
                // Store cal offset to flash
                const f32_bytes = std.mem.toBytes(result.resistance);
                std.mem.copyForwards(u8, flash_buff[0..], f32_bytes[0..]);
                rp2xxx.flash.range_program(flash_target_offset, flash_buff[0..]);

                return .{ .zero_probes_response = .{ .resistance_offset = result.resistance } };
            },
            .read_voltage_request => {
                const vout = read_voltmeter();
                return .{ .read_voltage_response = .{ .voltage = vout } };
            },
            .read_resistance_request => {
                const result = read_resistancemeter(true, undefined);
                const selected_resistor_value = get_resistor_value(result.bank);
                return .{ .read_resistance_response = .{ .resistance = result.resistance, .selected_resistor_value = selected_resistor_value } };
            },
            .read_pd_request => {
                const req_v = usb_pd.readRequestedVoltageMv() catch 0;
                const req_c = usb_pd.readRequestedCurrentMa() catch 0;
                const meas_v = usb_pd.readVoltageMv() catch 0;
                const meas_c = usb_pd.readCurrentMa() catch 0;

                return .{ .read_pd_response = .{
                    .requested_voltage_mv = req_v,
                    .requested_current_ma = req_c,
                    .measured_voltage_mv = meas_v,
                    .measured_current_ma = meas_c,
                } };
            },
            .set_psu_output_request => |request| {
                const is_enabled: u1 = @intFromBool(request.state);
                switch (request.channel) {
                    1 => {
                        pins.ch1_enable.put(is_enabled);
                        is_enabled_ch1 = request.state;
                    },
                    2 => {
                        pins.ch2_enable.put(is_enabled);
                        is_enabled_ch2 = request.state;
                    },
                    else => {},
                }
                return .{ .set_psu_output_response = .{ .status = 200 } };
            },
            .read_power_monitor_request => {
                var v_ch1 = pm_ch1.readVoltage() catch 0.0;
                var v_ch2 = pm_ch2.readVoltage() catch 0.0;
                var c_ch1 = pm_ch1.readCurrent() catch 0.0;
                var c_ch2 = pm_ch2.readCurrent() catch 0.0;

                if (!is_enabled_ch1) {
                    v_ch1 = 0.0;
                    c_ch1 = 0.0;
                }
                if (!is_enabled_ch2) {
                    v_ch2 = 0.0;
                    c_ch2 = 0.0;
                }

                return .{
                    .read_power_monitor_response = .{ .channel_1 = .{
                        .voltage = v_ch1,
                        .current = c_ch1,
                        .current_limit = current_limit_ch1,
                        .requested_voltage = requested_voltage_ch1,
                        .is_enabled = is_enabled_ch1,
                    }, .channel_2 = .{
                        .voltage = v_ch2,
                        .current = c_ch2,
                        .current_limit = current_limit_ch2,
                        .requested_voltage = requested_voltage_ch2,
                        .is_enabled = is_enabled_ch2,
                    } },
                };
            },
            .configure_psu_request => |request| {
                const c: u32 = if (request.channel == 1) 1 else 0;
                const channel: MCP47FEB22.Channel = @enumFromInt(c);
                const dac_val = voltage_to_dac(request.voltage);

                if (request.channel == 1) {
                    current_limit_ch1 = request.current_limit;
                    requested_voltage_ch1 = request.voltage;
                } else {
                    current_limit_ch2 = request.current_limit;
                    requested_voltage_ch2 = request.voltage;
                }

                dac.set_voltage(channel, dac_val) catch |err| return handle_err("DAC", err);
                return .{ .configure_psu_response = .{ .status = 200 } };
            },
            else => {},
        }
    }
    return .{ .error_response = .{ .message = "Error reading custom request" } };
}

fn handle_err(msg: []const u8, err: anytype) messages.ResponseMessage.kind_union {
    const err_msg = std.fmt.bufPrint(&err_msg_buff, "{s}: {any}", .{ msg, err }) catch "err";
    return .{ .error_response = .{ .message = err_msg } };
}

// --------------------------------------------------------------------------
// Resistance Logic
// --------------------------------------------------------------------------

const ResistorBank = enum {
    R_100_OHM,
    R_1K_OHM,
    R_10K_OHM,
    R_100K_OHM,
};

const resistance_result = struct {
    resistance: f32,
    bank: ResistorBank,
};

fn read_resistancemeter(calibrated: bool, opt_bank: ?ResistorBank) resistance_result {
    enable_resistancemeter();
    rp2xxx.adc.select_input(pins.resistance_adc);

    if (opt_bank) |bank| {
        const measured_sample = measure_bank_sample(bank);
        const calculated_resistance = calculate_measured_resistance(bank, measured_sample, calibrated);
        disable_all_circuits();
        return .{ .bank = bank, .resistance = calculated_resistance };
    }

    var best_sample: u12 = 0;
    var best_sample_diff: u12 = 4095;
    var best_bank: ResistorBank = .R_100K_OHM;

    // Sweep all banks to find the best one
    for (std.enums.values(ResistorBank)) |bank| {
        const measured_sample = measure_bank_sample(bank);

        // Calculate distance from target sample (halfway point)
        const value_diff = if (measured_sample > target_sample)
            measured_sample - target_sample
        else
            target_sample - measured_sample;

        if (best_sample_diff > value_diff) {
            best_sample_diff = value_diff;
            best_sample = measured_sample;
            best_bank = bank;
        }
    }

    const calculated_resistance = calculate_measured_resistance(best_bank, best_sample, calibrated);
    disable_all_circuits();
    return .{ .bank = best_bank, .resistance = calculated_resistance };
}

fn measure_bank_sample(bank: ResistorBank) u12 {
    // Set Mux Pins
    switch (bank) {
        .R_100_OHM => {
            pins.resistor_mux_a1.put(0);
            pins.resistor_mux_a0.put(1);
            rp2xxx.time.sleep_ms(5);
        },
        .R_1K_OHM => {
            pins.resistor_mux_a1.put(0);
            pins.resistor_mux_a0.put(0);
            rp2xxx.time.sleep_ms(5);
        },
        .R_10K_OHM => {
            pins.resistor_mux_a1.put(1);
            pins.resistor_mux_a0.put(0);
            rp2xxx.time.sleep_ms(15);
        },
        .R_100K_OHM => {
            pins.resistor_mux_a1.put(1);
            pins.resistor_mux_a0.put(1);
            rp2xxx.time.sleep_ms(100);
        },
    }

    // Flush first couple of readings
    rp2xxx.adc.start(.free_running);
    _ = rp2xxx.adc.read_result() catch 1;
    _ = rp2xxx.adc.read_result() catch 1;

    return get_sample_average(100) catch 1;
}

fn calculate_measured_resistance(bank: ResistorBank, sample: u12, calibrated: bool) f32 {
    const r_bank = get_resistor_value(bank);

    // Convert ADC sample (u12) to V_ADC (f32)
    const v_adc: f32 = (@as(f32, @floatFromInt(sample)) / 4095.0) * vin;

    // Check for short circuit
    if (v_adc < 0.00000001) return 0.0;

    const voltage_drop_r_bank = vin - v_adc;
    const r_mux_actual: f32 = 5.0;
    const r_top_total = r_bank + r_mux_actual;

    // Calculate Rx
    const total_bottom_resistance = r_top_total * (v_adc / voltage_drop_r_bank);

    var r_x = total_bottom_resistance;

    if (calibrated) {
        const resistance_offset_bytes = flash_target_contents[0..4];
        const resistance_offset = std.mem.bytesToValue(f32, resistance_offset_bytes);
        r_x -= resistance_offset;
        return if (r_x < 0.0) 0.0 else r_x;
    }

    return r_x;
}

fn get_resistor_value(bank: ResistorBank) f32 {
    return switch (bank) {
        .R_100_OHM => 100.0,
        .R_1K_OHM => 1000.0,
        .R_10K_OHM => 10000.0,
        .R_100K_OHM => 100000.0,
    };
}

// --------------------------------------------------------------------------
// Voltage Logic
// --------------------------------------------------------------------------

fn read_voltmeter() f32 {
    enable_voltmeter();
    rp2xxx.adc.select_input(pins.voltmeter_adc);

    const sample = get_sample_average(100) catch 1;
    const vout: f32 = (@as(f32, @floatFromInt(sample)) / 4095.0) * 3.3;

    const v_in = vout * scale_factor * 1.01;
    disable_all_circuits();
    return v_in;
}

// --------------------------------------------------------------------------
// PSU & PD Logic
// --------------------------------------------------------------------------

fn check_current_limits() void {
    const c_ch1 = pm_ch1.readCurrent() catch 5.0;
    const c_ch2 = pm_ch2.readCurrent() catch 5.0;

    if (c_ch1 > current_limit_ch1) {
        pins.ch1_enable.put(0);
        is_enabled_ch1 = false;
    }
    if (c_ch2 > current_limit_ch2) {
        pins.ch2_enable.put(0);
        is_enabled_ch2 = false;
    }
}

fn voltage_to_dac(voltage: f32) u16 {
    const clamped_voltage: f32 = @max(V_OUT_MIN, @min(V_OUT_MAX, voltage));
    const voltage_range: f32 = V_OUT_MAX - V_OUT_MIN;
    const dac_range: f32 = @floatFromInt(DAC_MAX - DAC_MIN);

    const dac_value: u16 = @intFromFloat(((V_OUT_MAX - clamped_voltage) * 0.99) * (@divFloor(dac_range, voltage_range)));
    return @max(DAC_MIN, @min(DAC_MAX, dac_value));
}

fn set_max_pd_voltage() !void {
    var index: usize = 0;
    var pdo_max: AP33772S.SRC_PDO = undefined;

    // Scan for highest voltage capability
    for (1..14) |i| {
        const source_pdo = try usb_pd.readSourcePDO(@intCast(i));
        if (source_pdo.type == 1) continue;
        if (source_pdo.voltage_max > pdo_max.voltage_max) {
            pdo_max = source_pdo;
            index = i;
        }
    }

    requestPDOAndVerify(.{
        .current_select = pdo_max.current_max_code,
        .pdo_index = @intCast(index),
        .voltage_select = 0x00,
    }, 250) catch {
        // Fallback if no PD source connected
        pdo_max.voltage_max = 50; // 5.0V encoded
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

// --------------------------------------------------------------------------
// Hardware Utilities
// --------------------------------------------------------------------------

fn disable_all_circuits() void {
    pins.voltmeter_switch.put(0);
    pins.resistancemeter_switch.put(0);
}

fn enable_voltmeter() void {
    disable_all_circuits();
    pins.voltmeter_switch.put(1);
    rp2xxx.time.sleep_ms(10);
}

fn enable_resistancemeter() void {
    disable_all_circuits();
    pins.resistancemeter_switch.put(1);
    rp2xxx.time.sleep_ms(10);
}

fn get_sample_average(sample_count: u32) !u12 {
    rp2xxx.adc.fifo.apply(rp2xxx.adc.fifo.Config{ .thresh = 0 });
    rp2xxx.adc.start(.free_running);

    var count: u32 = 0;
    var sum: u64 = 0;

    while (count < sample_count) {
        while (rp2xxx.adc.fifo.is_empty()) {}

        const batch_size = rp2xxx.adc.fifo.get_level();
        var i: u4 = 0;
        while (i < batch_size and count < sample_count) : (i += 1) {
            sum += try rp2xxx.adc.fifo.pop();
            count += 1;
        }
    }
    return @intCast(@divTrunc(sum, sample_count));
}
