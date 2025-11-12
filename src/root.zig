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
        const peripherals = microzig.chip.peripherals;
        const time = rp2xxx.time;
        const usb = rp2xxx.usb;

        // Variables and Buffers
        var shared_usb_rx_buff: [128]u8 = undefined;
        var shared_usb_tx_buff: [128]u8 = undefined;
        var usb_rx_buff: [128]u8 = undefined;
        var usb_tx_buff: [128]u8 = undefined;
        pub var usb_writer: std.Io.Writer = .fixed(&usb_tx_buff);
        pub const custom_usb_handler_type = ?*const fn (std.mem.Allocator, ProtoDef.RequestMessage) ProtoDef.ResponseMessage.kind_union;
        pub var custom_usb_handler: custom_usb_handler_type = null;

        // Separate spinlocks for RX and TX shared buffers
        const rx_spinlock = rp2xxx.multicore.Spinlock.init(0);
        const tx_spinlock = rp2xxx.multicore.Spinlock.init(1);

        var core1_stack: [4096]u32 = undefined;

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
            var driver = if (driver_enum == .driver) usb_cfg.lw_driver else return;
            driver.write(buff) catch {};
            _ = driver.writer_flush();
        }

        fn enable_fpu() void {
            var cpacr: u32 = microzig.cpu.peripherals.scb.CPACR;
            cpacr |= 0xF << 20; // Sets bits 20-23 (CP10 and CP11) to 0b11
            microzig.cpu.peripherals.scb.CPACR = cpacr;
            microzig.cpu.peripherals.fpu.FPCCR.modify(.{
                .ASPEN = 1,
                .LSPEN = 1,
            });
        }

        pub fn usbTxWrite(buff: []const u8) void {
            if (buff.len == 0) return;
            tx_spinlock.lock();
            defer tx_spinlock.unlock();

            // Send the response
            std.mem.copyForwards(u8, &shared_usb_tx_buff, buff);
            rp2xxx.multicore.fifo.write_blocking(@intCast(buff.len));
        }

        // --- Core 1 Logic (Protocol Handling) ---

        fn core1() void {
            enable_fpu();
            // 1. Setup allocator for protocol handler
            var buff: [256]u8 = undefined;
            var fba = std.heap.FixedBufferAllocator.init(&buff);
            const allocator = fba.allocator();

            while (true) {
                // Wait for usb data length in the fifo
                if (rp2xxx.multicore.fifo.read()) |message_len| {
                    handleProcessRx(allocator, message_len);
                    fba.reset();
                }
            }
        }

        fn handleProcessRx(allocator: std.mem.Allocator, message_len: u32) void {
            // Get the slice into a reader
            rx_spinlock.lock();
            const incoming_data = shared_usb_rx_buff[0..message_len];
            rx_spinlock.unlock();

            var reader: std.Io.Reader = std.Io.Reader.fixed(incoming_data);
            // Get the response kind
            protocol_handler.handle_incoming_usb(allocator, &reader, &usb_writer, ProtoDef, custom_usb_handler) catch |err| {
                var err_buff: [64]u8 = undefined;
                const formatted_err = std.fmt.bufPrint(err_buff[0..], "{any}", .{err}) catch "format error";
                const response = ProtoDef.ResponseMessage{ .kind = .{ .error_response = .{ .message = formatted_err } } };
                response.encode(&usb_writer, allocator) catch {};
            };

            // Copy it over to the shared tx buffer
            usbTxWrite(usb_writer.buffered());
            _ = usb_writer.consumeAll();
        }

        // --- Core 0 Logic (USB Polling) ---

        pub fn handleUsbRx() void {
            // Read in any USB data if there is any
            const rx_data = usb_cfg.lw_driver.read();

            if (rx_data.len > 0) {
                defer usb_cfg.lw_driver.reader_reset();
                // Copy the data to the shared memory
                rx_spinlock.lock();
                std.mem.copyForwards(u8, &shared_usb_rx_buff, rx_data);
                // Write the request to the webui
                usb_write(.webui, rx_data);
                rx_spinlock.unlock();

                // Signal to core1 data is ready and what its length it
                rp2xxx.multicore.fifo.write_blocking(@intCast(rx_data.len));
            }
        }

        pub fn handleUsbTx() void {
            // Writes a response to the fifo if it gets any
            if (rp2xxx.multicore.fifo.read()) |len| {
                tx_spinlock.lock();
                defer tx_spinlock.unlock();
                usb_write(.driver, shared_usb_tx_buff[0..len]);
                usb_write(.webui, shared_usb_tx_buff[0..len]);
            }
        }

        pub fn setup() void {
            // Initialize the USB device
            const usb_dev = usb.Usb(.{});
            usb_dev.init_clk();
            usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

            // Start the 2nd core
            rp2xxx.multicore.launch_core1_with_stack(&core1, &core1_stack);
        }

        pub fn run() void {
            // Initialize the USB device
            const usb_dev = usb.Usb(.{});
            usb_dev.init_clk();
            usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

            // Start the 2nd core
            rp2xxx.multicore.launch_core1_with_stack(&core1, &core1_stack);

            while (true) {
                handleUsbRx();
                handleUsbTx();
            }
        }
    };
}
