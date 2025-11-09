const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = @import("microzig").hal; // Changed for consistency

const types = microzig.core.usb.types;
const utils = microzig.core.usb.utils;

const bos = utils.BosConfig;
const DescType = types.DescType;

// --- MS/WebUSB vendor codes and descriptors ---
const MS_VENDOR_CODE: u8 = 0x20;
const WEBUSB_VENDOR_CODE: u8 = 0x30;
const WEBUSB_LANDING_PAGE_INDEX: u8 = 1;
const winusb_compat_id_descriptor = [_]u8{
    0x28, 0x00, 0x00, 0x00, // dwLength
    0x00, 0x01, // bcdVersion 1.0
    0x04, 0x00, // wIndex = Extended Compat ID Descriptor
    0x01, // one function
    0x00, 0x00, 0x00, 0x00, // reserved
    0x00, // interface number 0
    0x01, // reserved
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0, // Compatible ID
    0, 0, 0, 0, 0, 0, 0, 0, // Sub Compatible ID
    0, 0, 0, 0, 0, 0, // reserved
};
// ---

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

            if (ep_addr == self.ep_out) {
                // --- PING-PONG ECHO (RECEIVE) ---
                // Data has arrived. `data` is a slice with the received bytes.
                self.epout_primed = false; // It's no longer primed, it's been used.

                // --- FIX: Explicitly copy received data ---
                // This ensures self.echo_buf *definitely* contains
                // the data, just in case `data` is a temporary slice
                // or `self.echo_buf` isn't written yet.
                if (data.len > self.echo_buf.len) {
                    // This should never happen, but good to guard.
                    // We can't handle this, just drop it.
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
                // This logic is correct
                rp2xxx.rom.reset_to_usb_boot();
                // We do NOT re-prime ep_boot, because the device is rebooting.
            } else if (ep_addr == self.ep_in) {
                // --- PING-PONG ECHO (SEND COMPLETE) ---
                // The IN transfer (our echo) has completed.

                // 1. Mark the IN endpoint as no longer busy.
                self.epin_busy = false;

                // The buffer we just sent is now free.
                // Note: 'data.len' here is the length of the packet *we just sent*.
                const sent_len = data.len;
                self.echo_buf_len = 0; // Mark buffer as "consumed"

                // --- ZLP condition ---
                if (sent_len > 0 and sent_len == usb.max_packet_size) {
                    // We just sent a *full* packet.
                    // We must send a ZLP to terminate the transfer.
                    self.epin_busy = true;
                    self.device.?.endpoint_transfer(self.ep_in, &.{});
                    // The *next* time this transfer handler is called (for the ZLP),
                    // sent_len will be 0, epin_busy will be set to false,
                    // and ep_out will be re-primed (if not already).
                } else {
                    // We just sent a short packet (or a ZLP), so the transfer is over.
                    // 2. Now that the echo is done, re-prime ep_out
                    //    to receive the *next* packet, *if not already primed*.
                    if (!self.epout_primed and !self.epin_busy) {
                        self.epout_primed = true;
                        self.device.?.endpoint_transfer(self.ep_out, &self.echo_buf);
                    }
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
