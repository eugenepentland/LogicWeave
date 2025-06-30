const std = @import("std");
const zig_serial = @import("serial");
const modbus = @import("modbus");
const protobuf = @import("protobuf");
const definitions = @import("proto_gen/all.pb.zig");

pub fn set_usb_boot(port_name: []const u8) !void {
    var serial = try openDevice(port_name);
    defer serial.close();

    try zig_serial.configureSerialPort(serial, zig_serial.SerialConfig{
        .baud_rate = 115200,
        .word_size = .eight,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });

    const allocator = std.heap.page_allocator;
    var msg = definitions.AppMessage{
        .kind = definitions.AppMessage.kind_union{
            .usb_bootloader_request = .{},
        },
    };

    // Encode to bytes
    const encoded = try msg.encode(allocator);
    defer allocator.free(encoded);

    _ = try serial.writer().writeAll(encoded);
    std.log.info("Successfully put into boot mode", .{});
}

fn print_response(allocator: std.mem.Allocator, response: []u8) !void {
    const msg = try protobuf.pb_decode(definitions.AppMessage, response, allocator);
    switch (msg.kind.?) {
        inline else => |resp| {
            std.log.info("Message Recieved: {}", .{resp});
        },
    }
}

pub fn echo_message(port_name: []const u8) !void {
    var serial = try openDevice(port_name);
    defer serial.close();

    try zig_serial.configureSerialPort(serial, zig_serial.SerialConfig{
        .baud_rate = 115200,
        .word_size = .eight,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });
    const allocator = std.heap.page_allocator;
    // Wrap the Ping message inside AppMessage.kind_union
    var msg = definitions.AppMessage{
        .kind = definitions.AppMessage.kind_union{
            .echo_message = .{
                .message = protobuf.ManagedString.static("Recieved a message!"),
            },
        },
    };

    // Encode to bytes
    const encoded = try msg.encode(allocator);
    defer allocator.free(encoded);

    _ = try serial.writer().writeAll(encoded[0..]);

    // Wait for a response
    var buffer: [128]u8 = undefined;
    const resp_len = try serial.reader().read(buffer[0..]);
    std.log.info("{any}", .{buffer[0..resp_len]});
    const app_msg: definitions.AppMessage = try protobuf.pb_decode(definitions.AppMessage, buffer[0..resp_len], allocator);
    switch (app_msg.kind.?) {
        .echo_message => |echo| {
            std.log.info("Echoed Message: {s}", .{echo.message.getSlice()});
        },
        else => {},
    }
}

pub fn get_status(port_name: []const u8) !void {
    var serial = try openDevice(port_name);
    defer serial.close();

    try zig_serial.configureSerialPort(serial, zig_serial.SerialConfig{
        .baud_rate = 115200,
        .word_size = .eight,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });
    const allocator = std.heap.page_allocator;
    // Wrap the Ping message inside AppMessage.kind_union
    var msg = definitions.AppMessage{
        .kind = definitions.AppMessage.kind_union{
            .get_status_request = .{},
        },
    };

    // Encode to bytes
    const encoded = try msg.encode(allocator);
    defer allocator.free(encoded);

    _ = try serial.writer().writeAll(encoded[0..]);

    // Wait for a response
    var buffer: [128]u8 = undefined;
    const resp_len = try serial.reader().read(buffer[0..]);
    std.log.info("{any}", .{buffer[0..resp_len]});
    const app_msg: definitions.AppMessage = try protobuf.pb_decode(definitions.AppMessage, buffer[0..resp_len], allocator);
    switch (app_msg.kind.?) {
        .get_status_response => |response| {
            std.log.info("Echoed Message: {any}", .{response});
        },
        else => {},
    }
}

pub fn openDevice(port_name: []const u8) !std.fs.File {
    var retry_attempts: usize = 0;
    const max_total_delay_ms: usize = 5000; // Maximum wait time of 2 seconds
    const base_delay_ms: usize = 200; // Start with 100ms for the first retry
    var delay_ms: usize = base_delay_ms;

    while (retry_attempts < 20) {
        // Attempt to open the serial port to the device
        const device = std.fs.cwd().openFile(port_name, .{ .mode = .read_write }) catch {
            // Device not found, implement exponential backoff
            if (retry_attempts == 0) {
                std.debug.print("Device not found on port {s}, retrying.", .{port_name});
            } else {
                std.debug.print(".", .{});
            }

            // Sleep for the calculated delay
            std.time.sleep(std.time.ns_per_ms * 200);

            // Increase the delay exponentially for the next attempt
            delay_ms += 200;
            if (delay_ms > max_total_delay_ms) {
                delay_ms = max_total_delay_ms; // Cap the delay at 2 seconds
            }

            retry_attempts += 1;
            continue;
        };
        std.debug.print("\n", .{});
        // Return the device if successfully opened
        return device;
    }

    // Return an error if the device is not found after retries
    return error.NoDeviceFound;
}

pub fn write_firmware_to_device(port_name: []const u8, uf2_file_path: []const u8) !void {
    // Open the UF2 file
    const uf2_file = std.fs.cwd().openFile(uf2_file_path, .{}) catch {
        return error.NoFirmwareFound;
    };
    defer uf2_file.close();

    // Validate file size (UF2 file should be in 512 byte blocks)
    const uf2_stat = try uf2_file.stat();
    if ((uf2_stat.size % 512) != 0) {
        std.log.warn("{s} does not have a size multiple of 512. might be corrupt!", .{uf2_file_path});
    }

    const total_blocks = uf2_stat.size / 512;
    std.log.info("Total firmware size: {d} bytes", .{uf2_stat.size});

    // Open the serial port to the device
    var device = try openDevice(port_name);
    defer device.close();

    // Flash the firmware by writing UF2 file blocks to the device
    try uf2_file.seekTo(0); // Start at the beginning of the file
    var block_num: u64 = 0;
    var block: [512]u8 = undefined;

    std.debug.print("Flashing firmware to the device...", .{});
    while (true) {
        const rd_len = try uf2_file.read(&block);
        if (rd_len == 0) break; // End of file

        if (rd_len != block.len) {
            std.log.warn("Incomplete block read: Expected 512, got {d} bytes at block {d}", .{ rd_len, block_num });
            return error.IncompleteFile;
        }

        const wr_len = try device.write(&block);
        if (wr_len != block.len) {
            std.log.warn("Failed to write block {d}: Only {d} bytes written!", .{ block_num, wr_len });
            return error.WriteFailed;
        }
        block_num += 1;
        if ((total_blocks / 10) % block_num == 0) {
            std.debug.print(".", .{});
        }
    }
    std.debug.print("\n", .{});
    std.log.info("Successfully flashed {s}!", .{uf2_file_path});
}

pub fn main() !void {
    const port_name = if (@import("builtin").os.tag == .windows) "\\\\.\\COM1" else "/dev/ttyACM0";
    if (true) {
        const uf2_file_path = "zig-out/firmware/main_pico2_arm.uf2";

        // Sets the device to switch to the USB bootloader
        set_usb_boot(port_name) catch |err| {
            std.log.err("Couldn't open bootloader: {any}", .{err});
        };

        // Flashes the new firmware to the device
        write_firmware_to_device("/dev/sda", uf2_file_path) catch |err| {
            std.log.err("Couldn't write the firmware: {any}", .{err});
        };
    }

    // Write a test message

}
