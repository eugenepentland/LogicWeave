pub const protocol_handler = @import("protocol_handler.zig");

pub fn init(comptime ProtoDefType: anytype) type {
    // Aliases required by the module's implementation
    const std = @import("std");
    const microzig = @import("microzig");
    const rp2xxx = microzig.hal;

    // Import your external modules here
    const usb_cfg = @import("usb_config.zig");

    // Define the module structure that the user will import
    return struct {
        // --- Comptime Configured Message Type ---
        // This makes the message type available to all functions in this struct.
        pub const ProtoDef = ProtoDefType;

        // --- Standard Public/Internal Constants and Variables ---

        pub const protobuf = @import("protobuf");
        pub const Interrupt = @import("interrupts.zig");
        const peripherals = microzig.chip.peripherals;
        const time = rp2xxx.time;
        const usb = rp2xxx.usb;

        // Variables and Buffers
        var shared_usb_rx_buff: [128]u8 = undefined;
        var shared_usb_tx_buff: [128]u8 = undefined;
        var usb_rx_buff: [128]u8 = undefined;
        var usb_tx_buff: [128]u8 = undefined;
        pub var usb_writer: std.Io.Writer = .fixed(&usb_tx_buff);
        pub var custom_usb_handler: ?*const fn (std.mem.Allocator, ProtoDef.AppMessage, *std.Io.Writer) void = null;
        const shared_data_spinlock = rp2xxx.multicore.Spinlock.init(0);
        var stack: [8192]u32 = undefined;

        // --- Helpers ---

        fn panic(_: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
            @breakpoint();
            rp2xxx.rom.reboot_to_bootsel_now() catch {};
            while (true) {}
        }

        const cdc_driver = enum {
            driver,
            webui,
        };

        fn usb_write(driver_enum: cdc_driver, buff: []const u8) void {
            var driver = if (driver_enum == .driver) usb_cfg.cdc0_driver else usb_cfg.cdc1_driver;
            var write_buff = buff;

            const msg_lenth: u8 = @intCast(write_buff.len);
            _ = driver.write(&[_]u8{msg_lenth});

            while (write_buff.len > 0) {
                write_buff = driver.write(write_buff);
            }
            _ = driver.write_flush();
        }

        fn usb_read() []const u8 {
            var total_read: usize = 0;
            var read_buff: []u8 = usb_rx_buff[0..];

            // Read in usb data through the main driver
            while (true) {
                const len = usb_cfg.cdc0_driver.read(read_buff);
                read_buff = read_buff[len..];
                total_read += len;
                if (len == 0) break;
            }
            return usb_rx_buff[0..total_read];
        }

        pub fn usbTxWrite(buff: []const u8) void {
            if (buff.len == 0) return;
            shared_data_spinlock.lock();
            defer shared_data_spinlock.unlock();

            // Send the response
            std.mem.copyForwards(u8, &shared_usb_tx_buff, buff);
            rp2xxx.multicore.fifo.write_blocking(@intCast(buff.len));
        }

        // --- Core 1 Logic (Protocol Handling) ---

        fn core1() void {
            // 1. Setup allocator for protocol handler
            var buff: [512]u8 = undefined;
            var fba = std.heap.FixedBufferAllocator.init(&buff);
            const allocator = fba.allocator();

            while (true) {
                // Wait for usb data length in the fifo
                handleProcessRx(allocator);
            }
        }

        fn handleProcessRx(allocator: std.mem.Allocator) void {
            if (rp2xxx.multicore.fifo.read()) |message_len| {

                // Get the slice into a reader
                shared_data_spinlock.lock();
                const incoming_data = shared_usb_rx_buff[0..message_len];

                var reader: std.Io.Reader = std.Io.Reader.fixed(incoming_data);
                // Get the response kind
                protocol_handler.handle_incoming_usb(allocator, &reader, &usb_writer, ProtoDef, custom_usb_handler) catch |err| {
                    var err_buff: [64]u8 = undefined;
                    const formatted_err = std.fmt.bufPrint(err_buff[0..], "Handle Err: {any}", .{err}) catch "format error";
                    const response = ProtoDef.AppMessage{ .kind = .{ .error_response = .{ .message = formatted_err } } };
                    response.encode(&usb_writer, allocator) catch {};
                };

                shared_data_spinlock.unlock();

                // Copy it over to the shared tx buffer
                usbTxWrite(usb_writer.buffered());
                usb_writer.end = 0;
            }
        }

        // --- Core 0 Logic (USB Polling) ---

        fn handleUsbRx() void {
            // Read in any USB data if there is any
            const rx_data = usb_read();
            if (rx_data.len > 0) {
                if (rx_data.len == 2 and rx_data[1] == 1) rp2xxx.rom.reset_to_usb_boot();
                // Copy the data to the shared memory
                shared_data_spinlock.lock();
                std.mem.copyForwards(u8, &shared_usb_rx_buff, rx_data[1..]); //Ignore the first byte, its a length prefix
                // Write the request to the webui
                usb_write(.webui, rx_data[1..]);
                shared_data_spinlock.unlock();

                // Signal to core1 data is ready and what its length it
                rp2xxx.multicore.fifo.write_blocking(@intCast(rx_data.len - 1));
            }
        }

        fn handleUsbTx() void {
            // Writes a response to the fifo if it gets any
            if (rp2xxx.multicore.fifo.read()) |len| {
                shared_data_spinlock.lock();
                if (len == 1) rp2xxx.rom.reset_to_usb_boot();

                defer shared_data_spinlock.unlock();
                usb_write(.driver, shared_usb_tx_buff[0..len]);
                usb_write(.webui, shared_usb_tx_buff[0..len]);
            }
        }

        pub fn run() void {
            // Initialize the USB device
            const usb_dev = usb.Usb(.{});
            usb_dev.init_clk();
            usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

            // Start the 2nd core
            rp2xxx.multicore.launch_core1_with_stack(&core1, &stack);

            // Run the main loop
            // Main loop
            while (true) {
                // Poll for USB events
                usb_dev.task(false) catch unreachable;

                handleUsbRx();
                handleUsbTx();
            }
        }
    };
}
