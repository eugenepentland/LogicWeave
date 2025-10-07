const std = @import("std");
const zig_serial = @import("serial");
const protobuf = @import("protobuf");
const messages = @import("messages");

pub fn main() !void {
    // A temporary allocator is needed for this example.
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    const uf2_file_path = try get_firmware_path(allocator);
    std.log.info("{s}", .{uf2_file_path});

    // Put the device into the USB bootloader
    if (try get_device_port(allocator)) |port_name| {
        set_usb_boot(port_name, allocator) catch |err| {
            std.log.err("Couldn't open bootloader: {any}", .{err});
        };
    } else {
        std.log.info("No RP2040/RP2350 found to put into bootloader", .{});
    }

    // Flashes the new firmware to the device
    write_firmware_to_device("/dev/sda", uf2_file_path) catch |err| {
        std.log.err("Couldn't write the firmware: {any}", .{err});
    };
}

pub fn get_device_port(allocator: std.mem.Allocator) !?[]const u8 {
    // 1. Define the target VID and PID (hex is common for these IDs)
    const target_vid: u16 = 0x1E8B;
    const target_pid: u16 = 0x0001;

    // 2. Initialize the platform-specific iterator (InformationIterator)
    var it = try zig_serial.InformationIterator.init();
    defer it.deinit();

    // 3. Iterate over all available port information
    while (try it.next()) |port_info| {
        // 4. Check for a match
        if (port_info.vid == target_vid and port_info.pid == target_pid) {
            return try allocator.dupe(u8, port_info.system_location);
        }
    }

    // 6. No matching device found
    return null;
}

fn get_firmware_path(allocator: std.mem.Allocator) ![]u8 {
    var args = std.process.args();
    _ = args.next(); // Skip program name

    var file_name: []const u8 = "";

    while (args.next()) |arg| {
        // Check if the current argument is the "--file" flag
        if (std.mem.eql(u8, arg, "--file")) {
            // Get the *next* argument, which should be the file path.
            file_name = args.next() orelse {
                std.debug.print("Error: --file flag requires a value\n", .{});
                return error.MissingFileArgument;
            };
        }
    }

    if (std.mem.eql(u8, file_name, "")) {
        std.debug.print("Error: No file path provided\n", .{});
        return error.NoFileArgument;
    }

    return try std.mem.concat(allocator, u8, &[_][]const u8{ "zig-out/firmware/", file_name, ".uf2" });
}

pub fn set_usb_boot(port_name: []const u8, allocator: std.mem.Allocator) !void {
    var serial = try openDevice(port_name);
    defer serial.close();

    try zig_serial.configureSerialPort(serial, zig_serial.SerialConfig{
        .baud_rate = 115200,
        .word_size = .eight,
        .parity = .none,
        .stop_bits = .one,
        .handshake = .none,
    });

    const msg = messages.AppMessage{
        .kind = messages.AppMessage.kind_union{
            .usb_bootloader_request = .{ .val = 1 },
        },
    };

    // Encode to bytes
    var buff: [32]u8 = undefined;
    var writer = std.Io.Writer.fixed(&buff);
    try protobuf.encode(&writer, allocator, msg);

    var serial_writer = serial.writer(&.{});
    _ = try serial_writer.interface.writeAll(writer.buffered());
    std.log.info("Successfully put into boot mode", .{});
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
            std.Thread.sleep(std.time.ns_per_ms * 200);

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
