// src/protocol_handler.zig
const std = @import("std");
const microzig = @import("microzig");
const root = @import("root.zig");
const protobuf = @import("protobuf");
const firmware_config = @import("firmware_config");
// Import our new modules
const usb_cfg = @import("usb_config.zig");
const peripherals = microzig.chip.peripherals;
const SIO = peripherals.SIO;
const chip = rp2xxx.compatibility.chip;

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const Duration = microzig.drivers.time.Duration;
const has_rp2350b = rp2xxx.compatibility.has_rp2350b;

// --- Public Hardware Control Functions ---
pub inline fn getGPIO(gpio_enum: u32) rp2xxx.gpio.Pin {
    const gpio_num: u6 = @intCast(gpio_enum);
    return rp2xxx.gpio.num(gpio_num);
}

// In main.zig or pwm_utils.zig
const pwm = microzig.hal.pwm;
const pins = microzig.hal.pins;

// A wrapper struct to hold the slice and channel together
pub const PwmInfo = struct {
    slice: pwm.Slice,
    channel: pwm.Channel,
};

// This function replicates the core logic from pins.zig to find the PWM mapping.
pub fn get_pwm(gpio_num: u6) ?rp2xxx.pwm.Pwm {
    const channel: rp2xxx.pwm.Channel = if (gpio_num % 2 == 0) .a else .b;
    var slice_int: u6 = 0;

    if (0 <= gpio_num and gpio_num <= 31) {
        const offset = gpio_num % 16;
        slice_int = offset / 2;
    } else if (32 <= gpio_num and gpio_num <= 47) {
        const base_slice = 8;
        const offset = (gpio_num - 32) % 8;
        slice_int = base_slice + offset / 2;
    } else {
        return null;
    }

    return .{
        .slice_number = @intCast(slice_int),
        .channel = channel,
    };
}

/// Converts a physical GPIO pin number to its corresponding ADC Input channel.
pub fn gpio_to_adc_input(pin_num: u8) !rp2xxx.adc.Input {
    if (has_rp2350b) {
        // RP2350B: ADC pins are GPIO40 through GPIO47
        if (pin_num >= 40 and pin_num <= 47) {
            const channel_num = pin_num - 40;
            return @as(rp2xxx.adc.Input, @enumFromInt(channel_num));
        } else {
            return error.NotAnAdcPin;
        }
    } else {
        // RP2040 / RP2350A: ADC pins are GPIO26 through GPIO29
        if (pin_num >= 26 and pin_num <= 29) {
            const channel_num = pin_num - 26;
            return @as(rp2xxx.adc.Input, @enumFromInt(channel_num));
        } else {
            return error.NotAnAdcPin2;
        }
    }
}

pub inline fn read_direction(gpio: rp2xxx.gpio.Pin) rp2xxx.gpio.Direction {
    const mask = gpio.mask();

    switch (chip) {
        .RP2040 => {
            // Read the main GPIO_OE register.
            const current_oe = SIO.GPIO_OE.raw;

            if ((current_oe & mask) != 0) {
                return .out;
            } else {
                return .in;
            }
        },
        .RP2350 => {
            const current_oe = if (gpio.is_upper())
                // Use the high register for upper pins (GPIO24-GPIO31).
                SIO.GPIO_HI_OE.raw
            else
                // Use the low register for lower pins (GPIO0-GPIO23).
                SIO.GPIO_OE.raw;

            if ((current_oe & mask) != 0) {
                return .out;
            } else {
                return .in;
            }
        },
    }
}

fn process_request(
    allocator: std.mem.Allocator,
    comptime Context: type, // This holds ProtoDef, device, custom_handler, etc.
    message: Context.ProtoDef.RequestMessage,
) !Context.ProtoDef.ResponseMessage.kind_union {
    // Alias ProtoDef for brevity inside the function
    const ProtoDef = Context.ProtoDef;
    // Match on the kind of message
    if (message.kind) |kind_enum| {
        switch (kind_enum) {
            .firmware_info_request => |_| {
                return .{
                    .firmware_info_response = .{
                        .device = Context.device,
                        .version = firmware_config.version,
                        .serial_number = &Context.serial_number,
                    },
                };
            },
            .usb_bootloader_request => {
                // This is a special response that just writes a byte
                return .{ .gpio_read_response = .{ .state = true } };
            },
            .stream_uart_request => |request| {
                if (!request.enabled) {
                    Context.uart_stream_instance = null;
                    return .{ .stream_uart_response = .{ .data = &.{} } };
                }
                const tx_pin = getGPIO(request.tx_pin);
                const rx_pin = getGPIO(request.rx_pin);
                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));

                inline for (&.{ tx_pin, rx_pin }) |pin| {
                    pin.set_function(.uart);
                }

                try uart.apply_runtime(.{
                    .baud_rate = @intCast(request.baud_rate),
                    .clock_config = rp2xxx.clock_config,
                });
                Context.uart_stream_instance = uart;
                // Refactored to use explicit response constant
                return .{ .stream_uart_response = .{ .data = &.{} } };
            },
            .gpio_read_function_request => |request| {
                const pin: rp2xxx.gpio.Pin = @enumFromInt(request.gpio_pin);
                const regs = pin.get_regs();
                const func_selc = regs.ctrl.read().FUNCSEL;
                const function: ProtoDef.GpioFunction = switch (func_selc) {
                    .hstx => .hstx,
                    .spi => .spi,
                    .uart, .uart_alt => .uart,
                    .i2c => .i2c,
                    .pwm => .pwm,
                    .pio0, .pio1, .pio2 => .pio,
                    .gpck => .gpck,
                    .usb => .usb,
                    .sio => switch (read_direction(pin)) {
                        .in => .sio_in,
                        .out => .sio_out,
                    },
                    else => .none,
                };
                // Refactored to use explicit response constant
                return .{ .gpio_read_function_response = .{ .function = function } };
            },
            .gpio_read_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                const state = pin.read();
                // Refactored to use explicit response constant
                return .{ .gpio_read_response = .{ .state = state != 0 } };
            },
            .adc_read_request => |request| {
                const input = try gpio_to_adc_input(@intCast(request.gpio_pin));
                rp2xxx.adc.configure_gpio_pin_num(input);
                rp2xxx.adc.apply(.{});
                rp2xxx.adc.select_input(input);

                const sample = try rp2xxx.adc.convert_one_shot_blocking(input);
                // Refactored to use explicit response constant
                return .{ .adc_read_response = .{ .sample = @intCast(sample) } };
            },
            .gpio_write_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                pin.put(@intFromBool(request.state));
                // Refactored to use explicit response constant
                return .{ .gpio_write_response = .{ .status = 200 } };
            },
            .pwm_setup_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                pin.set_function(.pwm);
                const pwm_ch = get_pwm(@intCast(request.gpio_pin)) orelse {
                    return error.InvalidInput;
                };
                pwm_ch.slice().set_wrap(@intCast(request.wrap));
                if (request.clock_div_int > 0 or request.clock_div_frac > 0) {
                    pwm_ch.slice().set_clk_div(@intCast(request.clock_div_int), @intCast(request.clock_div_frac));
                }
                pwm_ch.slice().enable();
                // Refactored to use explicit response constant
                return .{ .pwm_setup_response = .{ .status = 200 } };
            },
            .pwm_set_level_request => |request| {
                const pwm_ch = get_pwm(@intCast(request.gpio_pin)) orelse {
                    return error.InvalidInput;
                };
                pwm_ch.set_level(@intCast(request.level));
                // Refactored to use explicit response constant
                return .{ .pwm_set_level_response = .{ .status = 200 } };
            },
            .gpio_function_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                switch (request.function) {
                    .sio_in => {
                        pin.set_function(.sio);
                        pin.set_direction(.in);
                    },
                    .sio_out => {
                        pin.set_function(.sio);
                        pin.set_direction(.out);
                    },
                    .none => {
                        pin.set_function(.disabled);
                    },
                    else => {}, // Handle other modes if necessary, or make this an error
                }
                // Refactored to use explicit response constant
                return .{ .gpio_function_response = .{ .status = 200 } };
            },
            .uart_setup_request => |request| {
                const tx_pin = getGPIO(request.tx_pin);
                const rx_pin = getGPIO(request.rx_pin);
                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));

                inline for (&.{ tx_pin, rx_pin }) |pin| {
                    pin.set_function(.uart);
                }

                try uart.apply_runtime(.{
                    .baud_rate = @intCast(request.baud_rate),
                    .clock_config = rp2xxx.clock_config,
                });
                // Refactored to use explicit response constant
                return .{ .uart_setup_response = .{ .status = 200 } };
            },
            .uart_write_request => |request| {
                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));
                const data = request.data;

                uart.write_blocking(data, Duration.from_ms(@intCast(request.timeout_ms))) catch {
                    uart.clear_errors();
                };
                // Refactored to use explicit response constant
                return .{ .uart_write_response = .{ .status = 200 } };
            },
            .uart_read_request => |request| {
                var buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));

                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));

                uart.read_blocking(buff, Duration.from_ms(@intCast(request.timeout_ms))) catch {
                    uart.clear_errors();
                };
                // Refactored to use explicit response constant
                return .{ .uart_read_response = .{ .data = buff[0..] } };
            },
            .spi_setup_request => |request| {
                const mosi_pin = getGPIO(request.mosi_pin);
                const miso_pin = getGPIO(request.miso_pin);
                const sclk_pin = getGPIO(request.sclk_pin);

                const spi_instance = rp2xxx.spi.instance.num(@intCast(request.instance_num));
                spi_instance.reset();

                inline for (&.{ mosi_pin, sclk_pin, miso_pin }) |pin| {
                    pin.set_function(.spi);
                }
                try spi_instance.apply(.{ .clock_config = rp2xxx.clock_config });

                // Refactored to use explicit response constant
                return .{ .spi_setup_response = .{ .status = 200 } };
            },
            .spi_write_request => |request| {
                const spi_instance = rp2xxx.spi.instance.num(@truncate(request.instance_num));

                if (request.cs_pin != 0) {
                    const cs_pin = getGPIO(request.cs_pin);
                    cs_pin.set_function(.sio);
                    cs_pin.set_direction(.out);
                    cs_pin.put(0);
                    spi_instance.write_blocking(u8, request.data);
                    cs_pin.put(1);
                } else {
                    spi_instance.write_blocking(u8, request.data);
                }

                // Refactored to use explicit response constant
                return .{ .spi_write_response = .{ .status = 200 } };
            },
            .spi_read_request => |request| {
                const buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));

                const spi_instance = rp2xxx.spi.instance.num(@truncate(request.instance_num));

                if (request.cs_pin != 0) {
                    const cs_pin = getGPIO(request.cs_pin);
                    cs_pin.set_function(.sio);
                    cs_pin.set_direction(.out);
                    cs_pin.put(0);
                    spi_instance.read_blocking(u8, @truncate(request.data), buff);
                    cs_pin.put(1);
                } else {
                    spi_instance.read_blocking(u8, @truncate(request.data), buff);
                }
                // Refactored to use explicit response constant
                return .{ .spi_read_response = .{ .data = buff } };
            },
            .i2c_setup_request => |request| {
                const sda_pin = getGPIO(request.sda_pin);
                const scl_pin = getGPIO(request.scl_pin);
                const ic2_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));

                inline for (&.{ scl_pin, sda_pin }) |pin| {
                    pin.set_slew_rate(.slow);
                    pin.set_schmitt_trigger_enabled(true);
                    pin.set_function(.i2c);
                }

                ic2_instance.apply(.{
                    .clock_config = rp2xxx.clock_config,
                });
                // Refactored to use explicit response constant
                return .{ .i2c_setup_response = .{ .status = 200 } };
            },
            .i2c_read_request => |request| {
                const buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));

                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);

                i2c_instance.read_blocking(@enumFromInt(device_address), buff, Duration.from_ms(100)) catch {};

                // Refactored to use explicit response constant
                return .{ .i2c_read_response = .{ .data = buff } };
            },
            .i2c_write_then_read_request => |request| {
                const buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));

                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);

                i2c_instance.write_then_read_blocking(@enumFromInt(device_address), request.data, buff, Duration.from_ms(100)) catch {};

                // Refactored to use explicit response constant
                return .{ .i2c_write_then_read_response = .{ .data = buff } };
            },
            .i2c_write_request => |request| {
                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);
                try i2c_instance.write_blocking(@enumFromInt(device_address), request.data, null);
                // Refactored to use explicit response constant
                return .{ .i2c_write_response = .{ .status = 200 } };
            },
            .gpio_pin_pull_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                switch (request.state) {
                    .PullUp => pin.set_pull(.up),
                    .PullDown => pin.set_pull(.down),
                    .None => pin.set_pull(.disabled),
                    else => return error.InvalidPullState,
                }
                // Refactored to use explicit response constant
                return .{ .gpio_pin_pull_response = .{ .status = 200 } };
            },
            .sleep_ms_request => |request| {
                rp2xxx.time.sleep_ms(@intCast(request.sleep_ms));
                // Refactored to use explicit response constant
                return .{ .sleep_ms_response = .{ .status = 200 } };
            },
            else => {
                if (Context.custom_usb_handler) |handler| {
                    return handler(allocator, message);
                }
            },
        }
    }
    return error.RequestNotFound;
}

pub fn handle_incoming_usb(
    allocator: std.mem.Allocator,
    reader: *std.Io.Reader,
    writer: *std.Io.Writer,
    comptime Context: type, // <--- Add this
    ProtoDef: type,
) !void {
    var msg = ProtoDef.RequestMessage.decode(reader, allocator) catch return error.ErrorDecoding;
    defer msg.deinit(allocator);

    const response_kind = process_request(allocator, Context, msg) catch @as(ProtoDef.ResponseMessage.kind_union, .{ .error_response = .{ .message = "error processing" } });
    const response = ProtoDef.ResponseMessage{ .kind = response_kind };
    response.encode(writer, allocator) catch return error.ErrorEncoding;
}
