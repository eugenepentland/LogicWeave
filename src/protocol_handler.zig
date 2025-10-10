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

// --- Public Hardware Control Functions ---
pub fn getGPIO(gpio_enum: u32) rp2xxx.gpio.Pin {
    const gpio_num: u6 = @intCast(gpio_enum);
    return rp2xxx.gpio.num(gpio_num);
}

pub fn encode_message(writer: *std.Io.Writer, allocator: std.mem.Allocator, kind: messages.AppMessage.kind_union) !void {
    try protobuf.encode(writer, allocator, messages.AppMessage{ .kind = kind });
}

pub fn handle_incoming_usb(allocator: std.mem.Allocator, reader: *std.Io.Reader, writer: *std.Io.Writer) !void {
    var msg = try protobuf.decode(messages.AppMessage, reader, allocator);
    defer msg.deinit(allocator);

    // Match on the kind of message
    if (msg.kind) |kind_enum| {
        switch (kind_enum) {
            .firmware_info_request => |_| {
                return encode_message(writer, allocator, .{ .firmware_info_response = .{
                    .hash = firmware_config.GIT_HASH,
                    .version = firmware_config.version,
                } });
            },
            .usb_bootloader_request => {
                rp2xxx.rom.reset_to_usb_boot();
                return;
            },
            .echo_message => |content| {
                return encode_message(writer, allocator, .{ .echo_message = .{ .message = content.message } });
            },
            .gpio_read_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                const state = pin.read();
                return encode_message(writer, allocator, .{ .gpio_read_response = .{ .state = state != 0 } });
            },
            .gpio_write_request => |request| {
                const pin = getGPIO(request.gpio_pin);
                pin.put(@intFromBool(request.state));
                return encode_message(writer, allocator, .{ .gpio_write_response = .{ .status = 200 } });
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
                    .pwm => {
                        pin.set_function(.pwm);
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
            .soft_spi_write_request => |request| {
                // Get GPIO pins from the request
                const cs_pin = getGPIO(request.cs_pin);
                const sclk_pin = getGPIO(request.sclk_pin);
                const mosi_pin = getGPIO(request.mosi_pin);
                const data_slice = request.data;

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

                return encode_message(writer, allocator, .{ .soft_spi_write_response = .{ .status = 200 } });
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

                i2c_instance.write_then_read_blocking(@enumFromInt(device_address), request.data, buff, Duration.from_ms(100)) catch {};

                return encode_message(writer, allocator, .{ .i2c_read_response = .{ .data = buff } });
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
            else => return,
        }
    }
}
