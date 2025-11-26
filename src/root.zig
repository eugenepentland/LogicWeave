pub const protocol_handler = @import("protocol_handler.zig");

pub fn init(comptime ProtoDefType: anytype, d: ProtoDefType.Device) type {
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
        pub const device = d;
        const peripherals = microzig.chip.peripherals;
        const time = rp2xxx.time;
        const usb = rp2xxx.usb;
        pub const usb_config = usb_cfg;

        // Variables and Buffers
        var shared_usb_rx_buff: [128]u8 = undefined;
        var tx_buff: [63]u8 = undefined;
        var tx_writer: std.Io.Writer = .fixed(&tx_buff);
        var shared_usb_tx_buff: [128]u8 = undefined;
        var usb_rx_buff: [128]u8 = undefined;
        var usb_tx_buff: [128]u8 = undefined;
        pub var uart_stream_instance: ?rp2xxx.uart.UART = undefined;
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

        const vendor_driver = enum {
            driver,
            webui,
        };

        pub fn usb_write(vd: vendor_driver, buff: []const u8) void {
            var driver = if (vd == .driver) &usb_cfg.web else &usb_cfg.web;
            driver.write(buff) catch return;
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
            //enable_fpu();
            // 1. Setup allocator for protocol handler
            var buff: [512]u8 = undefined;
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

            // --- MODIFIED CALL HERE ---
            // We pass @This() as the Context argument
            protocol_handler.handle_incoming_usb(allocator, &reader, &usb_writer, @This(), // <--- Pass the current Type here
                ProtoDef) catch |err| {
                var err_buff: [64]u8 = undefined;
                const formatted_err = std.fmt.bufPrint(err_buff[0..], "{any}", .{err}) catch "format error";
                const response = ProtoDef.ResponseMessage{ .kind = .{ .error_response = .{ .message = formatted_err } } };
                response.encode(&usb_writer, allocator) catch {};
            };

            // Copy it over to the shared tx buffer
            usbTxWrite(usb_writer.buffered());
            _ = usb_writer.consumeAll();
        }

        pub fn txProtobufMessage(allocator: std.mem.Allocator, kind: ProtoDef.ResponseMessage.kind_union) void {
            const response = ProtoDef.ResponseMessage{ .kind = kind };
            response.encode(&tx_writer, allocator) catch return;
            usb_write(.webui, tx_writer.buffered());
            _ = tx_writer.consumeAll();
        }

        // --- Core 0 Logic (USB Polling) ---

        pub fn handleUsbRx(vd: vendor_driver) void {
            var driver = if (vd == .driver) &usb_cfg.web else &usb_cfg.web;
            const rx_data_opt = driver.read();

            if (rx_data_opt) |rx_data| {
                // Moved DEFER here.
                // We MUST consume the packet to re-arm the USB endpoint,
                // even if the length is 0 or we don't process it.
                defer driver.read_consume();

                if (rx_data.len > 0) {
                    // Copy the data to the shared memory
                    rx_spinlock.lock();
                    std.mem.copyForwards(u8, &shared_usb_rx_buff, rx_data);
                    rx_spinlock.unlock();

                    // Signal to core1
                    rp2xxx.multicore.fifo.write_blocking(@intCast(rx_data.len));
                }
            }
        }

        pub fn handleUsbTx() void {
            // Writes a response to the fifo if it gets any
            if (rp2xxx.multicore.fifo.read()) |len| {
                tx_spinlock.lock();
                defer tx_spinlock.unlock();
                //usb_write(.driver, shared_usb_tx_buff[0..len]);
                usb_write(.webui, shared_usb_tx_buff[0..len]);
            }
        }

        pub fn setup() void {
            // Start the 2nd core
            rp2xxx.multicore.launch_core1_with_stack(&core1, &core1_stack);
        }

        pub fn run() void { // depriciated
            // Initialize the USB device
            const usb_dev = usb.Usb(.{});
            usb_dev.init_clk();
            usb_dev.init_device(&usb_cfg.DEVICE_CONFIGURATION) catch unreachable;

            // Start the 2nd core
            rp2xxx.multicore.launch_core1_with_stack(&core1, &core1_stack);

            while (true) {
                usb_dev.task(false) catch unreachable;

                handleUsbRx();
                handleUsbTx();
            }
        }
    };
}
