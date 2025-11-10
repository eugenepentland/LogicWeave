const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = @import("microzig").hal; // Changed for consistency

const types = microzig.core.usb.types;
const utils = microzig.core.usb.utils;

const bos = utils.BosConfig;
const DescType = types.DescType;

pub fn LogicWeaveDriver(comptime usb: anytype) type {
    return struct {
        device: ?types.UsbDevice = null,
        ep_in: u8 = 0,
        ep_out: u8 = 0,
        ep_boot: u8 = 0,
        // This buffer for the boot endpoint is correct.
        boot_buff: [1]u8 = undefined,

        // --- Simple "ping-pong" echo buffer ---
        echo_buf: [usb.max_packet_size]u8 = undefined,
        echo_buf_len: usize = 0,

        // Flag to track if an IN transfer is in progress
        epin_busy: bool = false,

        // --- FIX: Explicit flag to prevent double-priming ep_out ---
        epout_primed: bool = false,

        /// Called by the USB stack during device initialization
        fn init(ptr: *anyopaque, device: types.UsbDevice) void {
            var self: *@This() = @ptrCast(@alignCast(ptr));
            self.device = device;
        }

        /// Called by the USB stack when the host sets the configuration.
        fn open(ptr: *anyopaque, cfg: []const u8) !usize {
            var self: *@This() = @ptrCast(@alignCast(ptr));
            var curr_cfg = cfg;
            const start_len = cfg.len;

            // ... (Your descriptor parsing logic is correct) ...
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

                // This call resets the endpoint, which is good.
                self.device.?.endpoint_open(curr_cfg[0..desc_ep.length]);
                endpoints_found += 1;
                curr_cfg = bos.get_desc_next(curr_cfg);
            }

            if (self.ep_in == 0 or self.ep_out == 0 or self.ep_boot == 0) {
                return error.UsbDriverError;
            }

            // Reset buffer states
            self.echo_buf_len = 0;
            self.epin_busy = false;
            self.epout_primed = false; // Reset our new flag

            // Prime the OUT endpoint *only if* the IN endpoint isn't
            // (somehow) busy. This is the "kickstart" for the ping-pong.
            if (!self.epin_busy and !self.epout_primed) {
                // Tell the stack: "When data arrives on ep_out,
                // please write it into self.echo_buf."
                self.epout_primed = true; // Mark as primed
                self.device.?.endpoint_transfer(self.ep_out, &self.echo_buf);
            }

            // Prime ep_boot with its OWN dedicated buffer (This is correct)
            self.device.?.endpoint_transfer(self.ep_boot, &self.boot_buff);

            return start_len - curr_cfg.len;
        }

        /// Called for class/vendor-specific control requests.
        fn class_control(
            _: *anyopaque,
            _: types.ControlStage,
            _: *const types.SetupPacket,
        ) bool {
            // We don't need to handle any vendor requests for a simple echo
            return true;
        }

        /// This is the core logic. Called when data is received (OUT) or sent (IN).
        fn transfer(ptr: *anyopaque, ep_addr: u8, data: []u8) void {
            var self: *@This() = @ptrCast(@alignCast(ptr));
            std.log.info("Starting processing a transfer with ep_addr {d} data len {any}", .{ ep_addr, data.len });
            defer std.log.info("Finished processing a transfer with ep_addr {d} data len {any}", .{ ep_addr, data.len });
            if (ep_addr == self.ep_out) {
                self.epout_primed = false; // It's no longer primed, it's been used.

                if (data.len > self.echo_buf.len) {
                    // Re-prime ep_out and wait for the next packet.
                    if (!self.epout_primed) {
                        self.epout_primed = true;
                        self.device.?.endpoint_transfer(self.ep_out, &self.echo_buf);
                    }
                    return;
                }

                // Copy the received data from the 'data' slice into our
                // persistent 'echo_buf'.
                std.mem.copyForwards(u8, self.echo_buf[0..data.len], data);
                self.echo_buf_len = data.len;
                self.epin_busy = true;

                // Send the data *from* self.echo_buf
                self.device.?.endpoint_transfer(self.ep_in, self.echo_buf[0..self.echo_buf_len]);
            } else if (ep_addr == self.ep_boot) {
                rp2xxx.rom.reset_to_usb_boot();
            } else if (ep_addr == self.ep_in) {
                // IN echo done: free the buffer
                self.epin_busy = false;
                self.echo_buf_len = 0;

                // No ZLP. Immediately re-arm OUT for next packet.
                if (!self.epout_primed) {
                    self.epout_primed = true;
                    self.device.?.endpoint_transfer(self.ep_out, &self.echo_buf);
                }
            }
        }

        /// Returns the UsbClassDriver interface struct
        pub fn driver(self: *@This()) types.UsbClassDriver {
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
