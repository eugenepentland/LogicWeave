// src/protocol_handler.zig
const std = @import("std");
const microzig = @import("microzig");
const definitions = @import("proto_gen/all.pb.zig");
const protobuf = @import("protobuf");
const firmware_config = @import("firmware_config");
const Graphics = @import("graphics.zig");

// Import our new modules
const hardware = @import("hardware.zig");
const usb_cfg = @import("usb_config.zig");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;

var usb_rx_buff: [1024]u8 = undefined;

// --- USB Communication Functions ---
fn usb_cdc_read() []const u8 {
    var total_read: usize = 0;
    var read_buff: []u8 = usb_rx_buff[0..];

    while (true) {
        const len = usb_cfg.driver_cdc.read(read_buff);
        read_buff = read_buff[len..];
        total_read += len;
        if (len == 0) break;
    }
    return usb_rx_buff[0..total_read];
}

fn usb_cdc_write(buff: []const u8) void {
    const usb_dev = rp2xxx.usb.Usb(.{});
    var write_buff = buff;

    const msg_lenth: u8 = @intCast(write_buff.len);
    _ = usb_cfg.driver_cdc.write(&[_]u8{msg_lenth});

    while (write_buff.len > 0) {
        write_buff = usb_cfg.driver_cdc.write(write_buff);
        usb_dev.task(false) catch unreachable;
    }
    _ = usb_cfg.driver_cdc.write_flush();
    usb_dev.task(false) catch unreachable;
}

fn usb_cdc_write_protobuf(kind: definitions.AppMessage.kind_union, allocator: std.mem.Allocator) !void {
    const resp = definitions.AppMessage{ .kind = kind };
    const encoded = try resp.encode(allocator);
    defer allocator.free(encoded);
    usb_cdc_write(encoded);
}

// --- Main Public Handler Function ---
pub fn handle_incoming_usb(allocator: std.mem.Allocator) void {
    const rx_data = usb_cdc_read();

    if (rx_data.len > 0) {
        handleProto(allocator, rx_data[0..]) catch |err| {
            var err_buff: [64]u8 = undefined;
            const formatted_err = std.fmt.bufPrint(err_buff[0..], "{}", .{err}) catch "format error";
            usb_cdc_write_protobuf(.{ .error_response = .{ .message = protobuf.ManagedString.managed(formatted_err) } }, allocator) catch {};
        };
    }
}

fn handleProto(allocator: std.mem.Allocator, input: []const u8) !void {
    const msg = try protobuf.pb_decode(definitions.AppMessage, input, allocator);
    defer msg.deinit();

    // Match on the kind of message
    if (msg.kind) |kind_enum| {
        switch (kind_enum) {
            .firmware_info_request => |_| {
                try usb_cdc_write_protobuf(.{ .firmware_info_response = .{
                    .hash = protobuf.ManagedString.managed(firmware_config.GIT_HASH),
                    .version = protobuf.ManagedString.managed("1.0.1"),
                    .updated_at = protobuf.ManagedString.managed(firmware_config.UPDATED_AT),
                } }, allocator);
            },
            .write_text_request => |request| {
                try hardware.g.drawString(request.text.getSlice(), @intCast(request.x), @intCast(request.y));
                try usb_cdc_write_protobuf(.{ .write_text_response = .{ .status = 200 } }, allocator);
            },
            .clear_screen_request => |_| {
                hardware.g.clear(.White);
                try usb_cdc_write_protobuf(.{ .clear_screen_response = .{ .status = 200 } }, allocator);
            },
            .refresh_screen_request => |_| {
                hardware.screen.global_update(
                    Graphics.Graphics.getRotatedBuffer(hardware.g.old_frame_buffer)[0..],
                    Graphics.Graphics.getRotatedBuffer(hardware.g.frame_buffer)[0..],
                    .Fast,
                    0x19,
                );
                hardware.g.refreshFrameBuffer();
                try usb_cdc_write_protobuf(.{ .refresh_screen_response = .{ .status = 200 } }, allocator);
            },
            .usb_pd_enable_request => |request| {
                const pps = try hardware.getPPS(request.channel);
                pps.enable(request.on);
                try usb_cdc_write_protobuf(.{ .usb_pd_enable_response = .{ .status = 200 } }, allocator);
            },
            .usb_pd_write_pdo_request => |request| {
                const pps = try hardware.getPPS(request.channel);
                // Read in all of the PDO options
                for (1..14) |i| {
                    const pdo = try pps.readSourcePDO(@intCast(i));

                    if (pdo.is_pps()) {
                        // Check to see if the requested voltage is within the range the pps port accepts
                        if (request.voltage_mv > pdo.get_voltage_mv(false)) continue;
                        if (request.voltage_mv < pdo.get_voltage_min_mv(false)) continue;
                        if (request.current_limit_ma > pdo.get_current_ma()) continue;
                    } else {
                        if (request.voltage_mv != pdo.get_voltage_mv(false)) continue;
                        if (request.current_limit_ma > pdo.get_current_ma()) continue;
                    }

                    // PDO_INDEX: Use the current PDO index 'i'
                    const pdo_index: u16 = @intCast(i);

                    // VOLTAGE_SEL: Also crucial. "mV/100 for PPS, mV/200 for AVS".
                    // Assuming for now it's a PPS PDO for simplicity, so mV/100.
                    // You need to confirm if the PDO is PPS or AVS.
                    const voltage_sel: u16 = @intCast(request.voltage_mv / 100); // Assuming PPS for now.

                    // CURRENT_SEL: Derived from the requested current using the get_current_ma logic.
                    // We need to find the smallest current_max_code that results in a current
                    // equal to or greater than request.current_ma.
                    var current_sel: u16 = 0; // Initialize with the smallest possible code
                    while (true) {
                        const calculated_current_ma: u32 = 1000 + (@as(u32, current_sel) * 266);
                        if (calculated_current_ma >= request.current_limit_ma) {
                            break; // Found the smallest current_max_code that meets the requirement
                        }
                        current_sel += 1;
                        if (current_sel > 15) { // current_max_code is 4-bit, so max 15
                            current_sel = 15; // Cap at max if for some reason it goes above
                            break;
                        }
                    }

                    // Construct the pdo_request u16
                    const pdo_request_u16: u16 =
                        (pdo_index << 12) | // Bits 12-15
                        (current_sel << 8) | // Bits 8-11
                        (voltage_sel); // Bits 0-7

                    try pps.requestPDO(pdo_request_u16);
                    break;
                }

                try usb_cdc_write_protobuf(.{ .usb_pd_write_pdo_response = .{ .status = 200 } }, allocator);
            },
            .usb_pd_read_pdo_request => |request| {
                const pps = try hardware.getPPS(request.channel);

                const pdo = try pps.readSourcePDO(@intCast(request.index));

                try usb_cdc_write_protobuf(.{ .usb_pd_read_pdo_response = .{
                    .voltage_mv = pdo.get_voltage_mv(false),
                    .current_ma = pdo.get_current_ma(),
                    .is_fixed = pdo.type == 0,
                    .voltage_mv_min = pdo.get_voltage_min_mv(false),
                } }, allocator);
            },
            .usb_pd_read_request => |request| {
                const pps = try hardware.getPPS(request.channel);

                try usb_cdc_write_protobuf(.{ .usb_pd_read_response = .{
                    .measured_voltage_mv = try pps.readVoltageMv(),
                    .measured_current_ma = try pps.readCurrentMa(),
                    .requested_voltage_mv = try pps.readRequestedVoltageMv(),
                    .requested_current_ma = try pps.readRequestedCurrentMa(),
                    .on = pps.gate_open,
                } }, allocator);
            },
            .usb_bootloader_request => |_| {
                try usb_cdc_write_protobuf(.{ .usb_bootloader_response = .{ .status = 200 } }, allocator);
                rp2xxx.rom.reset_usb_boot(0, 0);
            },
            .echo_message => |content| {
                const message = content.message.getSlice();
                const resp: definitions.AppMessage.kind_union = .{ .echo_message = .{ .message = protobuf.ManagedString.managed(message) } };
                try usb_cdc_write_protobuf(resp, allocator);
            },
            .usb_pd_read_temperature_request => |_| {
                const temp1 = hardware.pps1.readTemperature() catch 0;
                const temp2 = hardware.pps1.readTemperature() catch 0;
                try usb_cdc_write_protobuf(.{ .usb_pd_read_temperature_response = .{
                    .temperature_ch1_c = temp1,
                    .temperature_ch2_c = temp2,
                } }, allocator);
            },
            .spi_setup_request => |request| {
                const mosi_pin = hardware.getGPIO(request.mosi_pin);
                //const miso_pin = getGPIO(request.miso_pin);
                const sclk_pin = hardware.getGPIO(request.sclk_pin);
                const spi_instance = rp2xxx.spi.instance.num(@intCast(request.instance_num));

                const cs_pin = hardware.getGPIO(request.cs_pin);
                cs_pin.set_function(.sio);
                cs_pin.set_direction(.out);
                cs_pin.put(1);

                inline for (&.{ mosi_pin, sclk_pin }) |pin| {
                    pin.set_function(.spi);
                }

                try spi_instance.apply(.{ .clock_config = rp2xxx.clock_config, .baud_rate = 2_000_000 });
                try usb_cdc_write_protobuf(.{ .spi_setup_response = .{ .status = 200 } }, allocator);
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
                    // FIX: Changed 'u3' to 'u4' to hold the value 8
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

                // --- End Bit-Banging ---

                try usb_cdc_write_protobuf(.{ .soft_spi_write_response = .{ .status = 200 } }, allocator);
            },
            .spi_read_request => |request| {
                var buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));
                defer allocator.free(buff);

                const cs_pin = hardware.getGPIO(request.cs_pin);
                cs_pin.set_function(.sio);
                cs_pin.set_direction(.out);
                cs_pin.put(0);
                defer cs_pin.put(1);

                const spi_instance = rp2xxx.spi.instance.num(@truncate(request.instance_num));
                spi_instance.transceive_blocking(u8, request.data.getSlice(), buff[0..]);
                const resp: definitions.AppMessage.kind_union = .{ .spi_read_response = .{ .data = protobuf.ManagedString.managed(buff[0..]) } };
                try usb_cdc_write_protobuf(resp, allocator);
            },
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

                try usb_cdc_write_protobuf(.{ .i2c_setup_response = .{ .status = 200 } }, allocator);
            },
            .i2c_read_request => |request| {
                var buff: []u8 = try allocator.alloc(u8, @truncate(request.byte_count));
                defer allocator.free(buff); // Handled by ManagedString in response
                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);

                try i2c_instance.write_then_read_blocking(@enumFromInt(device_address), request.data.getSlice(), buff[0..], null);

                try usb_cdc_write_protobuf(.{ .i2c_read_response = .{ .data = protobuf.ManagedString.managed(buff[0..]) } }, allocator);
            },
            .i2c_write_request => |request| {
                const i2c_instance = rp2xxx.i2c.instance.num(@truncate(request.instance_num));
                const device_address: u7 = @truncate(request.device_address);
                try i2c_instance.write_blocking(@enumFromInt(device_address), request.data.getSlice(), null);
                try usb_cdc_write_protobuf(.{ .i2c_write_response = .{ .status = 200 } }, allocator);
            },
            .gpio_read_request => |request| {
                const pin = hardware.getGPIO(request.gpio_pin);
                const state = pin.read();
                const kind: definitions.AppMessage.kind_union = .{ .gpio_read_response = .{ .state = state != 0 } };
                try usb_cdc_write_protobuf(kind, allocator);
            },
            .gpio_write_request => |request| {
                const pin = hardware.getGPIO(request.gpio_pin);
                pin.put(@intFromBool(request.state));
                try usb_cdc_write_protobuf(.{ .gpio_write_response = .{ .status = 200 } }, allocator);
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
                    else => {},
                }
                try usb_cdc_write_protobuf(.{ .gpio_mode_response = .{ .status = 200 } }, allocator);
            },
            .write_bank_voltage_request => |request| {
                try hardware.set_bank_voltage(request.bank, request.voltage);
                try usb_cdc_write_protobuf(.{ .write_bank_voltage_response = .{ .status = 200 } }, allocator);
            },
            else => {
                try usb_cdc_write_protobuf(.{ .error_response = .{ .message = protobuf.ManagedString.managed("Error with message") } }, allocator);
            },
        }
    }
}
