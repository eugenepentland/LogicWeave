// src/protocol_handler.zig
const std = @import("std");
const microzig = @import("microzig");
const definitions = @import("proto_gen/all.pb.zig");
const protobuf = @import("protobuf");
const firmware_config = @import("firmware_config");
const Graphics = @import("graphics.zig");
const fatfs = @import("zfat");

// Import our new modules
const hardware = @import("hardware.zig");
const usb_cfg = @import("usb_config.zig");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const Duration = microzig.drivers.time.Duration;

pub fn usb_cdc_write_protobuf(kind: definitions.AppMessage.kind_union, allocator: std.mem.Allocator) ![]const u8 {
    const resp = definitions.AppMessage{ .kind = kind };
    return try resp.encode(allocator);
}

fn calculateCurrentSelect(target_ma: u32) u4 {
    // These constants should be defined at a higher scope or within the device struct
    const CURRENT_BASE_MA: u32 = 1000; // 1.00A
    const CURRENT_STEP_MA: u32 = 266; // 266mA per step
    const MAX_CURRENT_SEL: u4 = 15;

    // Handle cases below the base current
    if (target_ma < CURRENT_BASE_MA) {
        return 0; // Or return an error, depends on desired behavior.
        // If 0 maps to 1000mA, then 0 is the minimum.
    }

    // Re-evaluate the loop as it is robust for this calculation:
    var current_sel: u4 = 0;
    while (true) {
        const calculated_current_ma: u32 = CURRENT_BASE_MA + (@as(u32, current_sel) * CURRENT_STEP_MA);
        if (calculated_current_ma >= target_ma) {
            break;
        }
        current_sel += 1;
        if (current_sel > MAX_CURRENT_SEL) {
            current_sel = MAX_CURRENT_SEL; // Cap at max
            break;
        }
    }
    return current_sel;
}

pub fn milliTo1dpFixed4(milli: u32) [4]u8 {
    const whole = milli / 1000; // assume whole < 100
    const tenths = (milli % 1000) / 100; // 0..9

    var out: [4]u8 = undefined;

    if (whole < 10) {
        // Use a space or invisible char instead of a leading '0'
        out[0] = ' '; // or 0xC2,0xA0 for non-breaking space, or 0xE2,0x80,0x8B for zero-width space
        out[1] = '0' + @as(u8, @intCast(whole));
    } else {
        out[0] = '0' + @as(u8, @intCast(whole / 10));
        out[1] = '0' + @as(u8, @intCast(whole % 10));
    }

    out[2] = '.';
    out[3] = '0' + @as(u8, @intCast(tenths));
    return out;
}

pub fn handle_incoming_usb(allocator: std.mem.Allocator, input: []const u8) ![]const u8 {
    const msg = try protobuf.pb_decode(definitions.AppMessage, input, allocator);
    defer msg.deinit();

    // Match on the kind of message
    if (msg.kind) |kind_enum| {
        switch (kind_enum) {
            .firmware_info_request => |_| {
                return try usb_cdc_write_protobuf(.{ .firmware_info_response = .{
                    .hash = protobuf.ManagedString.managed(firmware_config.GIT_HASH),
                    .version = protobuf.ManagedString.managed("1.0.1"),
                    .updated_at = protobuf.ManagedString.managed(firmware_config.UPDATED_AT),
                } }, allocator);
            },
            .echo_message => |content| {
                const message = content.message.getSlice();
                const resp: definitions.AppMessage.kind_union = .{ .echo_message = .{ .message = protobuf.ManagedString.managed(message) } };
                return try usb_cdc_write_protobuf(resp, allocator);
            },
            .gpio_read_request => |request| {
                const pin = hardware.getGPIO(request.gpio_pin);
                const state = pin.read();
                const kind: definitions.AppMessage.kind_union = .{ .gpio_read_response = .{ .state = state != 0 } };
                return try usb_cdc_write_protobuf(kind, allocator);
            },
            .gpio_write_request => |request| {
                const pin = hardware.getGPIO(request.gpio_pin);
                pin.put(@intFromBool(request.state));
                return try usb_cdc_write_protobuf(.{ .gpio_write_response = .{ .status = 200 } }, allocator);
            },
            .gpio_mode_request => |request| {
                const pin = hardware.getGPIO(request.gpio_pin);
                switch (request.mode) {
                    .input => {
                        pin.set_function(.sio);
                        pin.set_direction(.in);
                    },
                    .output => {
                        pin.set_function(.sio);
                        pin.set_direction(.out);
                    },
                    .pwm => {
                        pin.set_function(.pwm);
                    },
                    else => {}, // Handle other modes if necessary, or make this an error
                }
                return try usb_cdc_write_protobuf(.{ .gpio_mode_response = .{ .status = 200 } }, allocator);
            },
            .uart_setup_request => |request| {
                const tx_pin = hardware.getGPIO(request.tx_pin);
                const rx_pin = hardware.getGPIO(request.rx_pin);
                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));

                inline for (&.{ tx_pin, rx_pin }) |pin| {
                    pin.set_function(.uart);
                }

                try uart.apply_runtime(.{
                    .baud_rate = @intCast(request.baud_rate),
                    .clock_config = rp2xxx.clock_config,
                });
                return try usb_cdc_write_protobuf(.{ .uart_setup_response = .{ .status = 200 } }, allocator);
            },
            .uart_write_request => |request| {
                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));
                const data = request.data.getSlice();

                uart.write_blocking(data, Duration.from_ms(@intCast(request.timeout_ms))) catch {
                    uart.clear_errors();
                };
                return try usb_cdc_write_protobuf(.{ .uart_write_response = .{ .status = 200 } }, allocator);
            },
            .uart_read_request => |request| {
                const uart = rp2xxx.uart.instance.num(@intCast(request.instance_num));
                var buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));
                defer allocator.free(buff);

                uart.read_blocking(buff, Duration.from_ms(@intCast(request.timeout_ms))) catch {
                    uart.clear_errors();
                };
                const resp: definitions.AppMessage.kind_union = .{ .uart_read_response = .{ .data = protobuf.ManagedString.managed(buff[0..]) } };
                return try usb_cdc_write_protobuf(resp, allocator);
            },
            .spi_setup_request => |request| {
                const mosi_pin = hardware.getGPIO(request.mosi_pin);
                const miso_pin = hardware.getGPIO(request.miso_pin);
                const sclk_pin = hardware.getGPIO(request.sclk_pin);

                const spi_instance = rp2xxx.spi.instance.num(@intCast(request.instance_num));
                spi_instance.reset();

                const baud_rate: u32 = if (request.baud_rate > 0) request.baud_rate else 1_000_000;
                const peri_freq = comptime rp2xxx.clock_config.peri.?.frequency();
                try spi_instance.set_baudrate(baud_rate, peri_freq);

                inline for (&.{ mosi_pin, sclk_pin, miso_pin }) |pin| {
                    pin.set_function(.spi);
                }
                try spi_instance.apply(.{ .clock_config = rp2xxx.clock_config });
                time.sleep_us(100);
                return try usb_cdc_write_protobuf(.{ .spi_setup_response = .{ .status = 200 } }, allocator);
            },
            .soft_spi_write_request => |request| {
                // Get GPIO pins from the request
                const cs_pin = hardware.getGPIO(request.cs_pin);
                const sclk_pin = hardware.getGPIO(request.sclk_pin);
                const mosi_pin = hardware.getGPIO(request.mosi_pin);
                const data_slice = request.data.getSlice();

                // Configure all pins for software-controlled output
                inline for (&.{ cs_pin, mosi_pin, sclk_pin }) |pin| {
                    pin.set_function(.sio);
                    pin.set_direction(.out);
                }

                // Set initial pin states: CS is high (inactive), SCLK is low (idle for Mode 0)
                cs_pin.put(1);
                sclk_pin.put(0);

                // 1. Begin transaction by setting Chip Select low
                cs_pin.put(0);
                time.sleep_us(1); // Optional small delay for the peripheral to get ready

                // 2. Loop through each byte in the data slice
                for (data_slice) |byte_to_send| {
                    // 3. Loop through each bit in the byte, Most Significant Bit (MSB) first
                    var bit_index: u4 = 8;
                    while (bit_index > 0) {
                        bit_index -= 1;

                        // Isolate the MSB for the current iteration
                        const bit_to_send = (byte_to_send >> @intCast(bit_index)) & 1;

                        // Set the data line (MOSI) to the correct value
                        mosi_pin.put(@intCast(bit_to_send));

                        // Pulse the clock high. The slave device reads the MOSI value on this rising edge.
                        sclk_pin.put(1);
                        time.sleep_us(1); // This delay controls the clock's high-time (SPI speed)

                        // Bring the clock low again.
                        sclk_pin.put(0);
                        time.sleep_us(1); // This delay controls the clock's low-time (SPI speed)
                    }
                }

                // 4. End the transaction by setting Chip Select high
                cs_pin.put(1);

                return try usb_cdc_write_protobuf(.{ .soft_spi_write_response = .{ .status = 200 } }, allocator);
            },
            .spi_write_request => |request| {
                const spi_instance = rp2xxx.spi.instance.num(@truncate(request.instance_num));

                if (request.cs_pin != 0) {
                    const cs_pin = hardware.getGPIO(request.cs_pin);
                    cs_pin.set_function(.sio);
                    cs_pin.set_direction(.out);
                    cs_pin.put(0);
                    spi_instance.write_blocking(u8, request.data.getSlice());
                    cs_pin.put(1);
                } else {
                    spi_instance.write_blocking(u8, request.data.getSlice());
                }

                return try usb_cdc_write_protobuf(.{ .spi_write_response = .{ .status = 200 } }, allocator);
            },
            .spi_read_request => |request| {
                var buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));
                const spi_instance = rp2xxx.spi.instance.num(@truncate(request.instance_num));
                defer allocator.free(buff);

                if (request.cs_pin != 0) {
                    const cs_pin = hardware.getGPIO(request.cs_pin);
                    cs_pin.set_function(.sio);
                    cs_pin.set_direction(.out);
                    cs_pin.put(0);
                    spi_instance.read_blocking(u8, @truncate(request.data), buff[0..]);
                    cs_pin.put(1);
                } else {
                    spi_instance.read_blocking(u8, @truncate(request.data), buff[0..]);
                }

                const resp: definitions.AppMessage.kind_union = .{ .spi_read_response = .{ .data = protobuf.ManagedString.managed(buff[0..]) } };
                return try usb_cdc_write_protobuf(resp, allocator);
            },

            // Generic I2C functions (always included)
            .i2c_setup_request => |request| {
                const sda_pin = hardware.getGPIO(request.sda_pin);
                const scl_pin = hardware.getGPIO(request.scl_pin);
                const ic2_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));

                inline for (&.{ scl_pin, sda_pin }) |pin| {
                    pin.set_slew_rate(.slow);
                    pin.set_schmitt_trigger(.enabled);
                    pin.set_function(.i2c);
                }

                try ic2_instance.apply(.{
                    .clock_config = rp2xxx.clock_config,
                });

                return try usb_cdc_write_protobuf(.{ .i2c_setup_response = .{ .status = 200 } }, allocator);
            },
            .i2c_read_request => |request| {
                var buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));
                defer allocator.free(buff);
                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);

                try i2c_instance.write_then_read_blocking(@enumFromInt(device_address), request.data.getSlice(), buff[0..], null);

                return try usb_cdc_write_protobuf(.{ .i2c_read_response = .{ .data = protobuf.ManagedString.managed(buff[0..]) } }, allocator);
            },
            .i2c_write_request => |request| {
                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);
                try i2c_instance.write_blocking(@enumFromInt(device_address), request.data.getSlice(), null);
                return try usb_cdc_write_protobuf(.{ .i2c_write_response = .{ .status = 200 } }, allocator);
            },
            .gpio_self_test_request => |request| {
                const pin = hardware.getGPIO(request.gpio_pin);
                pin.set_function(.sio); // Set as GPIO
                pin.set_pull(.disabled);
                pin.set_drive_strength(.@"2mA");

                pin.set_direction(.out);
                pin.set_pull(.up);
                pin.put(1);
                rp2xxx.time.sleep_us(10);
                pin.set_direction(.in);

                const state_high = pin.read();
                pin.set_direction(.out);
                pin.set_pull(.down);
                pin.put(0);

                if (state_high != 1) {
                    return try usb_cdc_write_protobuf(.{ .gpio_self_test_response = .{ .result = .StuckLow } }, allocator);
                }

                rp2xxx.time.sleep_us(10);
                pin.set_direction(.in);
                const state_low = pin.read();

                pin.set_direction(.out);
                pin.set_pull(.down);
                pin.put(0);

                if (state_low != 0) {
                    return try usb_cdc_write_protobuf(.{ .gpio_self_test_response = .{ .result = .StuckHigh } }, allocator);
                }

                return try usb_cdc_write_protobuf(.{ .gpio_self_test_response = .{ .result = .Pass } }, allocator);
            },
            .gpio_pin_pull_request => |request| {
                const pin = hardware.getGPIO(request.gpio_pin);
                switch (request.state) {
                    .PullUp => pin.set_pull(.up),
                    .PullDown => pin.set_pull(.down),
                    .None => pin.set_pull(.disabled),
                    else => return error.InvalidPullState,
                }
                return try usb_cdc_write_protobuf(.{ .gpio_pin_pull_response = .{ .status = 200 } }, allocator);
            },

            // Feature-specific functions (conditionally compiled)
            // These will only be included if GENERIC_FIRMWARE is 'false'
            else => {
                if (comptime firmware_config.GENERIC) {
                    // For the generic firmware, all other messages are invalid
                    return error.InvalidMessage;
                } else {
                    // For the full-featured firmware, handle all other cases
                    switch (kind_enum) {
                        .sd_info_request => |_| {
                            try hardware.sd.initialize(20_000_000);
                            //const capactiy_bytes = (try hardware.sd.readCSD()).capacity_bytes;
                            return try usb_cdc_write_protobuf(.{ .sd_info_response = .{ .capacity_bytes = 10 } }, allocator);
                        },
                        .write_text_request => |request| {
                            try hardware.g.drawString(request.text.getSlice(), @intCast(request.x), @intCast(request.y));
                            return try usb_cdc_write_protobuf(.{ .write_text_response = .{ .status = 200 } }, allocator);
                        },
                        .clear_screen_request => |_| {
                            hardware.g.clear(.White);
                            return try usb_cdc_write_protobuf(.{ .clear_screen_response = .{ .status = 200 } }, allocator);
                        },
                        .refresh_screen_request => |_| {
                            try hardware.screen.global_update(
                                Graphics.Graphics.getRotatedBuffer(hardware.g.old_frame_buffer)[0..],
                                Graphics.Graphics.getRotatedBuffer(hardware.g.frame_buffer)[0..],
                                .Fast,
                                0x19,
                            );
                            hardware.g.refreshFrameBuffer();
                            return try usb_cdc_write_protobuf(.{ .refresh_screen_response = .{ .status = 200 } }, allocator);
                        },
                        .usb_pd_enable_request => |request| {
                            const pps = try hardware.getPPS(request.channel);
                            pps.enable(request.on);
                            return try usb_cdc_write_protobuf(.{ .usb_pd_enable_response = .{ .status = 200 } }, allocator);
                        },
                        .usb_pd_write_pdo_request => |request| {
                            const pps = try hardware.getPPS(request.channel);
                            var voltage_match = false;
                            var has_pps = false;
                            var max_current_ma: u32 = 0;

                            // Read in all of the PDO options
                            for (1..14) |i| {
                                const pdo = try pps.readSourcePDO(@intCast(i));

                                if (pdo.is_pps()) {
                                    has_pps = true;
                                    // Check to see if the requested voltage is within the range the pps port accepts
                                    if (request.voltage_mv > pdo.get_voltage_mv(false)) continue;
                                    if (request.voltage_mv < pdo.get_voltage_min_mv(false)) continue;
                                    //if (request.current_limit_ma > pdo.get_current_ma()) continue;
                                    if (request.voltage_mv % 100 != 0) continue;
                                } else {
                                    if (request.voltage_mv != pdo.get_voltage_mv(false)) continue;
                                }

                                voltage_match = true;
                                const pdo_max_current_ma = pdo.get_current_ma();
                                if (pdo_max_current_ma > max_current_ma) max_current_ma = pdo_max_current_ma;

                                if (request.current_limit_ma > pdo_max_current_ma) continue;

                                // PDO_INDEX: Use the current PDO index 'i'
                                const pdo_index: u4 = @intCast(i);

                                // VOLTAGE_SEL: Also crucial. "mV/100 for PPS, mV/200 for AVS".
                                // Assuming for now it's a PPS PDO for simplicity, so mV/100.
                                const voltage_sel: u8 = @intCast(request.voltage_mv / 100); // Assuming PPS for now.

                                // CURRENT_SEL: Derived from the requested current using the get_current_ma logic.
                                // We need to find the smallest current_max_code that results in a current
                                // equal to or greater than request.current_ma.
                                const current_sel: u4 = calculateCurrentSelect(request.current_limit_ma);

                                try pps.requestPDO(.{
                                    .pdo_index = pdo_index,
                                    .current_select = current_sel,
                                    .voltage_select = voltage_sel,
                                });

                                return try usb_cdc_write_protobuf(.{ .usb_pd_write_pdo_response = .{
                                    .pdo_index = pdo_index,
                                } }, allocator);
                            }

                            // Give an error saying you are just above the current limit
                            if (voltage_match) {
                                const errmsg = try std.fmt.allocPrint(allocator, "CH{d} Current limit set too high. Set to {s}V {s}A but the max is {s}V {s}A", .{
                                    request.channel,
                                    &milliTo1dpFixed4(request.voltage_mv),
                                    &milliTo1dpFixed4(request.current_limit_ma),
                                    &milliTo1dpFixed4(request.voltage_mv),
                                    &milliTo1dpFixed4(max_current_ma),
                                });
                                defer allocator.free(errmsg);
                                return try usb_cdc_write_protobuf(.{ .error_response = .{
                                    .message = protobuf.ManagedString.managed(errmsg),
                                } }, allocator);
                            } else {
                                var errmsg: []u8 = undefined;
                                if (has_pps) {
                                    const voltage_round_up = ((request.voltage_mv + 99) / 100) * 100;
                                    const voltage_round_down = (request.voltage_mv / 100) * 100;
                                    errmsg = try std.fmt.allocPrint(allocator, "CH{d} {d}mV Invalid Voltage. Try {d}mV or {d}mV instead.", .{
                                        request.channel,
                                        request.voltage_mv,
                                        voltage_round_up,
                                        voltage_round_down,
                                    });
                                } else {
                                    var buff = try allocator.alloc(u8, 48);
                                    var index: usize = 0;
                                    defer allocator.free(buff);
                                    for (1..14) |i| {
                                        const pdo = try pps.readSourcePDO(@intCast(i));
                                        if (pdo.is_pps()) continue;
                                        if (pdo.get_voltage_mv(false) == 0) continue;

                                        const slice = try std.fmt.bufPrint(buff[index..], "{s}V, ", .{milliTo1dpFixed4(pdo.get_voltage_mv(false))});
                                        index = index + slice.len;
                                    }
                                    errmsg = try std.fmt.allocPrint(allocator, "CH{d} {d}mV Invalid Voltage. Available voltages are {s}", .{
                                        request.channel,
                                        request.voltage_mv,
                                        buff[0..index],
                                    });
                                }
                                defer allocator.free(errmsg);

                                return try usb_cdc_write_protobuf(.{ .error_response = .{
                                    .message = protobuf.ManagedString.managed(errmsg),
                                } }, allocator);
                            }
                        },
                        .usb_pd_read_pdo_request => |request| {
                            const pps = try hardware.getPPS(request.channel);

                            const pdo = try pps.readSourcePDO(@intCast(request.index));

                            return try usb_cdc_write_protobuf(.{ .usb_pd_read_pdo_response = .{
                                .voltage_mv = pdo.get_voltage_mv(false),
                                .current_ma = pdo.get_current_ma(),
                                .is_fixed = pdo.type == 0,
                                .voltage_mv_min = pdo.get_voltage_min_mv(false),
                            } }, allocator);
                        },
                        .usb_pd_read_request => |request| {
                            const pps = try hardware.getPPS(request.channel);

                            return try usb_cdc_write_protobuf(.{ .usb_pd_read_response = .{
                                .measured_voltage_mv = try pps.readVoltageMv(),
                                .measured_current_ma = try pps.readCurrentMa(),
                                .requested_voltage_mv = try pps.readRequestedVoltageMv(),
                                .requested_current_ma = try pps.readRequestedCurrentMa(),
                                .on = pps.gate_open,
                            } }, allocator);
                        },
                        // doesn't work currently for the rp2350
                        .usb_bootloader_request => |_| {
                            return try usb_cdc_write_protobuf(.{ .usb_bootloader_response = .{ .status = 200 } }, allocator);
                            //rp2xxx.rom.reset_usb_boot(0, 0);
                        },
                        .usb_pd_read_temperature_request => |_| {
                            const temp1 = hardware.pps1.readTemperature() catch 0;
                            const temp2 = hardware.pps1.readTemperature() catch 0;
                            return try usb_cdc_write_protobuf(.{ .usb_pd_read_temperature_response = .{
                                .temperature_ch1_c = temp1,
                                .temperature_ch2_c = temp2,
                            } }, allocator);
                        },
                        .write_bank_voltage_request => |request| {
                            try hardware.set_bank_voltage(request.bank, request.voltage);
                            return try usb_cdc_write_protobuf(.{ .write_bank_voltage_response = .{ .status = 200 } }, allocator);
                        },
                        // If any other unhandled cases exist in the full-featured version, they would fall here.
                        // For now, we assume all are covered or explicitly invalid.
                        else => {
                            return error.InvalidMessage;
                        },
                    }
                }
            },
        }
    }
    return error.NoMessage;
}
