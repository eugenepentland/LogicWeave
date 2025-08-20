// src/main.zig
const std = @import("std");
const microzig = @import("microzig");
const Duration = microzig.drivers.time.Duration;
const firmware_config = @import("firmware_config");
const fatfs = @import("zfat");
const Graphics = @import("graphics.zig");
const peripherals = microzig.chip.peripherals;

// Import our new modules
const hardware = @import("hardware.zig");
const usb_cfg = @import("usb_config.zig");
const protocol_handler = @import("protocol_handler.zig");
const uart = rp2xxx.uart.instance.num(0);

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const usb = rp2xxx.usb;
const DEBOUNCE_DELAY_US: u64 = 5e3;
var bit_index: u32 = 0;
var atp_index: u32 = 0;
const time_time_ms = 9000;
var powered_on = false;
pub const microzig_options = microzig.Options{
    .log_level = .info,
    .logFn = rp2xxx.uart.logFn,
    .cpu = .{ .ram_vectors = true },
};

pub fn panic(msg: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    //std.log.err("The RP2350 has crashed: {s}. {any}", .{ message, s });
    hardware.g.clear(.White);
    hardware.g.drawString(msg, 5, 25) catch {};
    //try hardware.g.drawString("Initalized!", 5, 65);
    hardware.screen.global_update(Graphics.Graphics.getRotatedBuffer(hardware.g.old_frame_buffer)[0..], Graphics.Graphics.getRotatedBuffer(hardware.g.frame_buffer)[0..], .Fast, 0x19) catch {};
    @breakpoint();
    rp2xxx.rom.reset_usb_boot(0, 0);
    while (true) {}
}

pub fn main() !void {
    //std.log.info("booting up", .{});
    // 1. Setup allocator for protocol handler
    var buffer: [4096]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(buffer[0..]);
    const allocator = fba.allocator();
    var uart_buff: [512]u8 = undefined;

    // Initalize the screen, USB PD ports, intterupts for logicweave board
    hardware.init(allocator) catch {};

    hardware.init_buttons();

    hardware.sd.initialize(8_000_000) catch {
        return error.NoSDCardFound;
    };

    // tell ZFAT about our physical disk:
    fatfs.disks[0] = &hardware.sd.interface;
    try hardware.global_fs.mount("0:", true);
    defer fatfs.FileSystem.unmount("0:") catch |e| std.log.err("failed to unmount filesystem: {s}", .{@errorName(e)});
    // setup the power brick
    setup_power() catch {
        return error.ErrorSettingPower;
    };

    try hardware.g.drawString("DRC Programer", 5, 25);
    try hardware.g.drawString("Initalized!", 5, 65);

    try hardware.screen.global_update(Graphics.Graphics.getRotatedBuffer(hardware.g.old_frame_buffer)[0..], Graphics.Graphics.getRotatedBuffer(hardware.g.frame_buffer)[0..], .Fast, 0x19);
    hardware.g.refreshFrameBuffer();

    // 4. Main loop
    while (true) {
        if (hardware.ok_btn_pressed) {
            hardware.program_running = true;
            hardware.ok_btn_pressed = false;
            const msg = "1\n";

            try hardware.writeMessage("Init BIT Test", 5, 25);
            try power_on();
            try bypass_startup_error(msg);
            try read_until_end("BIT", msg, &uart_buff, "0:/bit.txt");
        } else if (hardware.l_btn_pressed) {
            hardware.program_running = true;
            hardware.l_btn_pressed = false;
            const msg = "2\n";
            try hardware.writeMessage("Init ATP Test", 5, 25);
            try power_on();
            try bypass_startup_error(msg);
            try read_until_end("ATP", msg, &uart_buff, "0:/atp.txt");
            //try hardware.writeMessage("ATP complete", 5, 25);
        } else if (hardware.r_btn_pressed) {
            hardware.program_running = true;
            hardware.r_btn_pressed = false;
            if (powered_on) try power_off() else try power_on();
            //try hardware.writeMessage("Clearing Test Counter Test", 5, 25);
        }
        hardware.program_running = false;
    }
}

fn setup_power() !void {
    try hardware.write_pdo_request(.{ .channel = 1, .voltage_mv = 15000, .current_limit_ma = 2500, .pdo_index = 0 });
    try hardware.write_pdo_request(.{ .channel = 2, .voltage_mv = 5000, .current_limit_ma = 2500, .pdo_index = 0 });
}

fn bypass_startup_error(msg: []const u8) !void {
    var one_byte: [1]u8 = undefined;
    var buff: [2]u8 = undefined; // This will store the last two bytes read

    try uart.write_blocking(msg, null);

    // Initialize buff with some values that won't match "\r\n"
    // (or just read the first byte before the loop, then the second inside)
    // Let's just make sure we read at least one byte before checking
    try uart.read_blocking(&one_byte, null);
    buff[0] = one_byte[0];

    // Read the second byte and then start the loop
    try uart.read_blocking(&one_byte, null);
    buff[1] = one_byte[0];

    // reads one byte at a time, maintaining a sliding window of the last two bytes
    while (!std.mem.eql(u8, "\r\n", &buff)) {
        // Shift the bytes in buff: the new byte goes into buff[1],
        // and the old buff[1] becomes buff[0] (effectively discarding the oldest byte)
        buff[0] = buff[1];
        try uart.read_blocking(&one_byte, null);
        buff[1] = one_byte[0];
    }
}

fn power_on() !void {
    const pps2 = try hardware.getPPS(2);
    pps2.enable(true);

    rp2xxx.time.sleep_ms(10);

    const pps1 = try hardware.getPPS(1);
    pps1.enable(true);
    powered_on = true;
    rp2xxx.time.sleep_ms(2000);
}

fn power_off() !void {
    const pps1 = try hardware.getPPS(1);
    pps1.enable(true);

    rp2xxx.time.sleep_ms(10);

    const pps2 = try hardware.getPPS(2);
    pps2.enable(true);
    powered_on = false;
}

fn read_until_end(name: []const u8, msg: []const u8, buffer: []u8, _: [:0]const u8) !void {
    // 1. Write the initial message to the UART to start the test.
    try uart.write_blocking(msg, null);

    var bytes_read: usize = 0; // Tracks the total bytes accumulated in `buffer`.
    const start_time = time.get_time_since_boot().to_us();
    var current_time = start_time;

    var progress_buff: [30]u8 = undefined; // Buffer for formatting status messages.
    var progress: []const u8 = undefined;

    var test_started_displayed: bool = false;
    var last_screen_update_time: u64 = 0;

    // --- SD Card and File Handling (Re-initialized on each loop) ---
    hardware.sd.initialize(8_000_000) catch {
        try hardware.writeMessage("Error: No SD Card", 5, 45);
        return error.NoSDCardFound;
    };
    // Open the file for appending.
    var file = try fatfs.File.open("0:/atp.txt", .{ .access = .read_write, .mode = .open_append });

    // Write ONLY the new chunk to the SD card.
    try file.writer().writeAll("\n\n");

    // Close the file immediately after writing.
    file.close();

    // Loop until the timeout is reached (480 seconds).
    while ((current_time - start_time) < 48e7) {
        current_time = time.get_time_since_boot().to_us();

        // Update the on-screen timer frequently.
        if ((current_time - last_screen_update_time) > 1e6) {
            progress = try std.fmt.bufPrint(&progress_buff, "Test Time {d}ms", .{(current_time - start_time) / 1000});
            try hardware.writeMessage(progress, 5, 25);
            last_screen_update_time = current_time;
        }

        // Define a chunk buffer for each read operation.
        var read_chunk_buffer: [2]u8 = undefined;

        // Try to read a blocking chunk of 32 bytes.
        uart.read_blocking(&read_chunk_buffer, Duration.from_ms(100)) catch |err| {
            if (err != error.Timeout) {
                uart.clear_errors();
            }
            continue; // A timeout is normal, just continue the loop.
        };

        // If read_blocking succeeds, we have a full 32-byte chunk.
        const bytes_in_chunk = 2;

        // Before appending to the memory buffer, check if there's enough space.
        if (bytes_read + bytes_in_chunk > buffer.len) {
            try hardware.writeMessage("Error: Small Buffer", 5, 45);
            return;
        }

        // --- SD Card and File Handling (Re-initialized on each loop) ---
        hardware.sd.initialize(8_000_000) catch {
            try hardware.writeMessage("Error: No SD Card", 5, 45);
            return error.NoSDCardFound;
        };
        // Open the file for appending.
        file = try fatfs.File.open("0:/atp.txt", .{ .access = .read_write, .mode = .open_append });

        // Write ONLY the new chunk to the SD card.
        try file.writer().writeAll(read_chunk_buffer[0..bytes_in_chunk]);

        // Close the file immediately after writing.
        file.close();

        // Copy the new chunk into the main memory buffer for token scanning.
        std.mem.copyForwards(u8, buffer[bytes_read..], read_chunk_buffer[0..bytes_in_chunk]);
        bytes_read += bytes_in_chunk;

        // Create a slice representing the valid data accumulated in memory so far.
        const current_data = buffer[0..bytes_read];

        // --- Token Checking ---
        // This logic remains the same, scanning the in-memory buffer.

        if (std.mem.indexOf(u8, current_data, "[END]") != null) {
            //try hardware.writeMessage("Test Finished", 5, 25);
            return;
        }

        if (!test_started_displayed and std.mem.indexOf(u8, current_data, "[START]") != null) {
            try hardware.writeMessage("Test Started", 5, 25);
            test_started_displayed = true;
        }

        if (std.mem.indexOf(u8, current_data, "[RESULT]")) |result_index| {
            progress = try std.fmt.bufPrint(&progress_buff, "{s} Test", .{name});
            hardware.g.clear(.White);
            try hardware.g.drawString(progress, 5, 25);

            const result_start_index = result_index + "[RESULT]".len;
            const result_payload_len = 4;

            if (current_data.len >= result_start_index + result_payload_len) {
                const result_end_index = result_start_index + result_payload_len;
                try hardware.g.drawString(current_data[result_start_index..result_end_index], 5, 65);

                try hardware.screen.global_update(Graphics.Graphics.getRotatedBuffer(hardware.g.old_frame_buffer)[0..], Graphics.Graphics.getRotatedBuffer(hardware.g.frame_buffer)[0..], .Fast, 0x19);
                hardware.g.refreshFrameBuffer();
                return;
            }
        }
    }

    // If the while loop exits, it means the timeout was reached.
    try hardware.writeMessage("Error: Read timeout", 5, 25);
}
