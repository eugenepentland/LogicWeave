const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = @import("microzig").hal;

const types = microzig.core.usb.types;
const utils = microzig.core.usb.utils;

const bos = utils.BosConfig;
const DescType = types.DescType;

pub fn LogicWeaveDriver(comptime usb: anytype) type {
    return struct {
        const Self = @This();

        device: ?types.UsbDevice = null,

        ep_in: u8 = 0,
        ep_out: u8 = 0,
        ep_boot: u8 = 0,

        // Simple fixed buffers, no ring
        tx_buf: [usb.max_packet_size]u8 = undefined,
        tx_len: u8 = 0,
        tx_busy: bool = false,

        rx_buf: [usb.max_packet_size]u8 = undefined,
        rx_len: u8 = 0,
        rx_ready: bool = false,

        boot_buf: [1]u8 = .{0},

        // ---------------------------------------------------------------------
        // Init / open
        // ---------------------------------------------------------------------

        fn init(ptr: *anyopaque, device: types.UsbDevice) void {
            var self: *Self = @ptrCast(@alignCast(ptr));
            self.device = device;
        }

        fn open(ptr: *anyopaque, cfg: []const u8) !usize {
            var self: *Self = @ptrCast(@alignCast(ptr));
            var curr_cfg = cfg;
            const start_len = cfg.len;

            self.ep_in = 0;
            self.ep_out = 0;
            self.ep_boot = 0;

            if (bos.try_get_desc_as(types.InterfaceDescriptor, curr_cfg)) |desc_itf| {
                if (desc_itf.interface_class != 0xFF) {
                    return types.DriverErrors.UnsupportedInterfaceClassType;
                }
            } else {
                return types.DriverErrors.ExpectedInterfaceDescriptor;
            }
            curr_cfg = bos.get_desc_next(curr_cfg);

            var endpoints_found: u8 = 0;
            while (endpoints_found < 3 and curr_cfg.len > 0 and bos.get_desc_type(curr_cfg) == DescType.Endpoint) {
                const desc_ep = bos.get_desc_as(types.EndpointDescriptor, curr_cfg);

                switch (types.Endpoint.dir_from_address(desc_ep.endpoint_address)) {
                    .In => self.ep_in = desc_ep.endpoint_address,
                    .Out => {
                        if (self.ep_out == 0) {
                            self.ep_out = desc_ep.endpoint_address;
                        } else {
                            self.ep_boot = desc_ep.endpoint_address;
                        }
                    },
                }

                self.device.?.endpoint_open(curr_cfg[0..desc_ep.length]);
                endpoints_found += 1;
                curr_cfg = bos.get_desc_next(curr_cfg);
            }

            if (self.ep_in == 0 or self.ep_out == 0 or self.ep_boot == 0) {
                return error.UsbDriverError;
            }

            // Clear state
            self.tx_len = 0;
            self.tx_busy = false;
            self.rx_len = 0;
            self.rx_ready = false;

            // Prime OUT endpoint: always ready to receive up to max_packet_size
            self.device.?.endpoint_transfer(self.ep_out, self.rx_buf[0..usb.max_packet_size]);

            // Prime boot endpoint with a tiny buffer (never &.{})
            self.device.?.endpoint_transfer(self.ep_boot, self.boot_buf[0..self.boot_buf.len]);

            return start_len - curr_cfg.len;
        }

        // ---------------------------------------------------------------------
        // Public API
        // ---------------------------------------------------------------------

        /// Queue a packet to EP_IN: [len][payload...], payload.len <= max_packet_size-1
        pub fn write(self: *Self, payload: []const u8) !void {
            var dev = self.device orelse return error.NotReady;
            if (!dev.ready()) return error.NotReady;
            if (self.ep_in == 0) return error.NoEndpoint;
            if (self.tx_busy) return error.Busy;

            if (payload.len > usb.max_packet_size - 1) {
                return error.MessageTooLong;
            }

            const n: u8 = @intCast(payload.len);
            self.tx_buf[0] = n;
            if (n > 0) {
                std.mem.copyForwards(u8, self.tx_buf[1 .. 1 + n], payload[0..n]);
            }

            self.tx_len = n + 1;
            self.tx_busy = true;

            // IMPORTANT: only send the bytes we actually filled
            dev.endpoint_transfer(self.ep_in, self.tx_buf[0..self.tx_len]);
        }

        /// Get the last received payload (without the length byte).
        /// Returns null if nothing new has arrived.
        pub fn read(self: *Self) ?[]const u8 {
            if (!self.rx_ready) return null;
            return self.rx_buf[0..self.rx_len];
        }

        /// Call after you're done handling the last rx packet.
        pub fn read_consume(self: *Self) void {
            self.rx_ready = false;
            self.rx_len = 0;

            // ADD THIS HERE:
            // Re-prime the endpoint now that the buffer is free.
            self.device.?.endpoint_transfer(self.ep_out, self.rx_buf[0..usb.max_packet_size]);
        }

        // ---------------------------------------------------------------------
        // Control
        // ---------------------------------------------------------------------

        fn class_control(
            _: *anyopaque,
            _: types.ControlStage,
            _: *const types.SetupPacket,
        ) bool {
            return true;
        }

        // ---------------------------------------------------------------------
        // Transfer callback
        // ---------------------------------------------------------------------

        fn transfer(ptr: *anyopaque, ep_addr: u8, data: []u8) void {
            var self: *Self = @ptrCast(@alignCast(ptr));

            if (ep_addr == self.ep_out) {
                // OUT data from host
                if (data.len > 0) {
                    const plen = data[0];

                    // Basic safety: plen must fit in packet and our buffer
                    if (plen <= usb.max_packet_size - 1 and plen <= data.len - 1) {
                        std.mem.copyForwards(u8, self.rx_buf[0..plen], data[1 .. 1 + plen]);
                        self.rx_len = plen;
                        self.rx_ready = true;
                    } else {
                        // Invalid length, drop
                        self.rx_len = 0;
                        // If it's invalid, we probably want to re-arm immediately to clear the stall,
                        // otherwise we just drop it and wait for app to consume empty.
                        // For simplicity, let's treat it as "ready but empty" so app consumes it.
                        self.rx_ready = true;
                    }
                }

                // DELETE THIS LINE FROM HERE:
                // self.device.?.endpoint_transfer(self.ep_out, self.rx_buf[0..usb.max_packet_size]);

                // The hardware will now automatically NAK incoming packets until we re-prime.

            } else if (ep_addr == self.ep_in) {
                // IN transfer completed; free to send the next one
                self.tx_busy = false;
            } else if (ep_addr == self.ep_boot) {
                // Any traffic on boot EP triggers USB boot
                rp2xxx.rom.reset_to_usb_boot();
            }
        }

        // ---------------------------------------------------------------------
        // Driver registration
        // ---------------------------------------------------------------------

        pub fn driver(self: *Self) types.UsbClassDriver {
            return .{
                .ptr = self,
                .fn_init = init,
                .fn_open = open,
                .fn_class_control = class_control,
                .fn_transfer = transfer,
            };
        }
    };
}
