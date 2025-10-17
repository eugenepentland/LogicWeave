// src/protocol_handler.zig
const std = @import("std");
const microzig = @import("microzig");
const messages = @import("proto_gen/all.pb.zig");
const protobuf = @import("protobuf");
const firmware_config = @import("firmware_config");
// Import our new modules
const usb_cfg = @import("usb_config.zig");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const Duration = microzig.drivers.time.Duration;
const has_rp2350b = rp2xxx.compatibility.has_rp2350b;

// --- Public Hardware Control Functions ---
pub fn getGPIO(gpio_enum: u32) rp2xxx.gpio.Pin {
    const gpio_num: u6 = @intCast(gpio_enum);
    return rp2xxx.gpio.num(gpio_num);
}

pub fn encode_message(writer: *std.Io.Writer, allocator: std.mem.Allocator, kind: messages.AppMessage.kind_union) !void {
    try protobuf.encode(writer, allocator, messages.AppMessage{ .kind = kind });
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

pub fn handle_incoming_usb(allocator: std.mem.Allocator, reader: *std.Io.Reader, writer: *std.Io.Writer) !void {
    var msg = try protobuf.decode(messages.AppMessage, reader, allocator);
    defer msg.deinit(allocator);

    // Match on the kind of message
    if (msg.kind) |kind_enum| {
        switch (kind_enum) {
            .firmware_info_request => |_| {
                const id_array = try getUniqueBoardId();
                var id_buff: [1]u8 = undefined;
                picoGetUniqueBoardIdString(&id_buff, id_array);
                return encode_message(writer, allocator, .{ .firmware_info_response = .{
                    .hash = firmware_config.GIT_HASH,
                    .version = firmware_config.version,
                    .serial_number = id_buff[0 .. id_buff.len - 1],
                } });
            },
            .usb_bootloader_request => {
                rp2xxx.rom.reset_to_usb_boot();
                return;
            },
            .gpio_read_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                const state = pin.read();
                return encode_message(writer, allocator, .{ .gpio_read_response = .{ .state = state != 0 } });
            },
            .adc_read_request => |request| {
                const input = try gpio_to_adc_input(@intCast(request.gpio_pin));
                rp2xxx.adc.configure_gpio_pin_num(input);
                rp2xxx.adc.apply(.{});
                rp2xxx.adc.select_input(input);

                const sample = try rp2xxx.adc.convert_one_shot_blocking(input);
                return encode_message(writer, allocator, .{ .adc_read_response = .{ .sample = @intCast(sample) } });
            },
            .gpio_write_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                pin.put(@intFromBool(request.state));
                return encode_message(writer, allocator, .{ .gpio_write_response = .{ .status = 200 } });
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
                return encode_message(writer, allocator, .{ .pwm_setup_response = .{ .status = 200 } });
            },
            .pwm_set_level_request => |request| {
                const pwm_ch = get_pwm(@intCast(request.gpio_pin)) orelse {
                    return error.InvalidInput;
                };
                pwm_ch.set_level(@intCast(request.level));
                return encode_message(writer, allocator, .{ .pwm_set_level_response = .{ .status = 200 } });
            },
            .gpio_mode_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                switch (request.mode) {
                    .input => {
                        pin.set_function(.sio);
                        pin.set_direction(.in);
                    },
                    .output => {
                        pin.set_function(.sio);
                        pin.set_direction(.out);
                    },
                    .disabled => {
                        pin.set_function(.disabled);
                    },
                    else => {}, // Handle other modes if necessary, or make this an error
                }
                return encode_message(writer, allocator, .{ .gpio_mode_response = .{ .status = 200 } });
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
                return encode_message(writer, allocator, .{ .uart_setup_response = .{ .status = 200 } });
            },
            .uart_write_request => |request| {
                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));
                const data = request.data;

                uart.write_blocking(data, Duration.from_ms(@intCast(request.timeout_ms))) catch {
                    uart.clear_errors();
                };
                return encode_message(writer, allocator, .{ .uart_write_response = .{ .status = 200 } });
            },
            .uart_read_request => |request| {
                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));
                var buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));

                uart.read_blocking(buff, Duration.from_ms(@intCast(request.timeout_ms))) catch {
                    uart.clear_errors();
                };
                return encode_message(writer, allocator, .{ .uart_read_response = .{ .data = buff[0..] } });
            },
            .spi_setup_request => |request| {
                const mosi_pin = getGPIO(request.mosi_pin);
                const miso_pin = getGPIO(request.miso_pin);
                const sclk_pin = getGPIO(request.sclk_pin);

                const spi_instance = rp2xxx.spi.instance.num(@intCast(request.instance_num));
                spi_instance.reset();

                //const baud_rate: u32 = if (request.baud_rate > 0) request.baud_rate else 1_000_000;
                //const peri_freq = comptime rp2xxx.clock_config.peri.?.frequency();
                //try spi_instance.set_baudrate(baud_rate, peri_freq);

                inline for (&.{ mosi_pin, sclk_pin, miso_pin }) |pin| {
                    pin.set_function(.spi);
                }
                try spi_instance.apply(.{ .clock_config = rp2xxx.clock_config });

                return encode_message(writer, allocator, .{ .spi_setup_response = .{ .status = 200 } });
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

                return encode_message(writer, allocator, .{ .spi_write_response = .{ .status = 200 } });
            },
            .spi_read_request => |request| {
                const buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));
                defer allocator.free(buff);

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

                return encode_message(writer, allocator, .{ .spi_read_response = .{ .data = buff } });
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

                return encode_message(writer, allocator, .{ .i2c_setup_response = .{ .status = 200 } });
            },
            .i2c_read_request => |request| {
                const buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));
                defer allocator.free(buff);

                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);

                i2c_instance.read_blocking(@enumFromInt(device_address), buff, Duration.from_ms(100)) catch {};

                return encode_message(writer, allocator, .{ .i2c_read_response = .{ .data = buff } });
            },
            .i2c_write_then_read_request => |request| {
                const buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));
                defer allocator.free(buff);

                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);

                i2c_instance.write_then_read_blocking(@enumFromInt(device_address), request.data, buff, Duration.from_ms(100)) catch {};

                return encode_message(writer, allocator, .{ .i2c_write_then_read_response = .{ .data = buff } });
            },
            .i2c_write_request => |request| {
                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);
                try i2c_instance.write_blocking(@enumFromInt(device_address), request.data, null);
                return encode_message(writer, allocator, .{ .i2c_write_response = .{ .status = 200 } });
            },
            .gpio_pin_pull_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                switch (request.state) {
                    .PullUp => pin.set_pull(.up),
                    .PullDown => pin.set_pull(.down),
                    .None => pin.set_pull(.disabled),
                    else => return error.InvalidPullState,
                }
                return encode_message(writer, allocator, .{ .gpio_pin_pull_response = .{ .status = 200 } });
            },
            .sleep_ms_request => |request| {
                rp2xxx.time.sleep_ms(@intCast(request.sleep_ms));
                return encode_message(writer, allocator, .{ .sleep_ms_response = .{ .status = 200 } });
            },
            else => return,
        }
    }
}

const rom = rp2xxx.rom;
const arch = rp2xxx.compatibility.arch;

const PICO_UNIQUE_BOARD_ID_SIZE_BYTES: u32 = 8;
const OTP_ROW_UNIQUE_ID: u32 = 0x1c;
const OTP_FLAG_READ: u32 = 0; // 0 for read, 1 for write
const SYS_INFO_CHIP_INFO: u32 = 0x0001;
const OUTPUT_BUFFER_WORD_SIZE: u32 = 9; // 9 u32 words as in the SDK

pub const UniqueIdError = error{
    RomFunctionNotFound,
    UnexpectedWordCount,
};

pub fn getUniqueBoardId() ![PICO_UNIQUE_BOARD_ID_SIZE_BYTES]u8 {
    var out: [OUTPUT_BUFFER_WORD_SIZE]u32 = undefined;
    var retrieved_id: [PICO_UNIQUE_BOARD_ID_SIZE_BYTES]u8 = undefined;

    // 1) Lookup + null-check
    const any_fn_opt = rom.lookup_and_cache_function(.get_sys_info);
    if (any_fn_opt == null) return UniqueIdError.RomFunctionNotFound;

    const func: *const rom.signatures.get_sys_info =
        @ptrCast(@alignCast(any_fn_opt.?));

    // 2) Call the ROM function
    const rc: i32 = func(&out, OUTPUT_BUFFER_WORD_SIZE, SYS_INFO_CHIP_INFO);

    // 3) Surface real ROM errors (negative)
    if (rc < 0) {
        // Turn ROM error codes into your rom.Error set (NotPermitted, InvalidArgument, etc.)
        return error.RomError;
    }

    // 4) Expect exactly 4 words for SYS_INFO_CHIP_INFO, same as SDK (assert(rc == 4))
    if (@as(u32, @intCast(rc)) != 4) return UniqueIdError.UnexpectedWordCount;

    // 5) Byte extraction identical to the SDK:
    // retrieved_id[i] = out.bytes[PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1 + 2*4 - i];
    const out_bytes: []const u8 = std.mem.asBytes(&out);
    //const base: usize = (2 * 4); // 8
    for (0..PICO_UNIQUE_BOARD_ID_SIZE_BYTES) |i| {
        retrieved_id[i] = out_bytes[PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1 + 2 * 4 - i];
    }

    return retrieved_id;
}
// Note: The board_id array size should be based on the constant PICO_UNIQUE_BOARD_ID_SIZE_BYTES
pub fn picoGetUniqueBoardIdString(id_out: []u8, board_id: [PICO_UNIQUE_BOARD_ID_SIZE_BYTES]u8) void {
    // C code's 'assert(len > 0);' is implicitly covered by id_out.len > 0 later,
    // but a check for a completely empty buffer is good practice.
    if (id_out.len == 0) return;

    // Calculate the maximum number of hex characters that *could* be written.
    const max_hex_chars = PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2;

    // Determine the actual loop limit: min(output buffer size - 1, max_hex_chars)
    // - id_out.len - 1: Reserves space for the null terminator.
    // - max_hex_chars: Prevents reading beyond the board_id.
    const loop_limit: usize = @min(id_out.len - 1, max_hex_chars);

    var i: usize = 0;
    while (i < loop_limit) : (i += 1) {
        // C code: (i&1) is 0 for even i (high nibble) and 1 for odd i (low nibble).
        // C code: (4 - 4 * (i&1))
        //   If i is even (i&1=0): 4 - 0 = 4 (shift right 4 for the high nibble)
        //   If i is odd (i&1=1): 4 - 4 = 0 (shift right 0 for the low nibble)

        const is_odd = i & 1;
        // Shift amount is 4 for the high nibble (i is even) or 0 for the low nibble (i is odd)
        const shift_amount = 4 - (4 * is_odd);

        // board_id[i/2] accesses the correct byte
        // The @as(u3, @intCast(shift_amount)) is correct for casting to a small enough int type for a shift
        const nibble = (board_id[i / 2] >> @as(u3, @intCast(shift_amount))) & 0x0F;

        // Convert the nibble (0-15) to hex character ('0'-'F').
        if (nibble < 10) {
            id_out[i] = @as(u8, @intCast(nibble + '0')); // More idiomatic cast
        } else {
            // 'A' - 10 is '7' (ASCII 55). nibble + 'A' - 10 is 'A' to 'F'
            id_out[i] = @as(u8, @intCast(nibble - 10 + 'A')); // More idiomatic cast
        }
    }

    // Always null-terminate the string at the end of the written characters.
    // i will be exactly loop_limit after the loop finishes.
    // This is safe because loop_limit <= id_out.len - 1, so i <= id_out.len - 1.
    id_out[i] = 0;
}
