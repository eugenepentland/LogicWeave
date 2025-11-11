const std = @import("std");
const io = std.io; // --- NEW: Alias for std.io
const microzig = @import("microzig");
const rp2xxx = @import("microzig").hal;

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
        boot_buff: [1]u8 = undefined,

        reader_buff: [usb.max_packet_size]u8 = undefined,
        writer_buff: [usb.max_packet_size]u8 = undefined,

        // --- CHANGED: Correct type and initialized to undefined ---
        reader: io.Reader = undefined,
        writer: io.Writer = undefined,

        epin_busy: bool = false,
        epout_primed: bool = false,

        /// Called by the USB stack during device initialization
        fn init(ptr: *anyopaque, device: types.UsbDevice) void {
            var self: *@This() = @ptrCast(@alignCast(ptr));
            self.device = device;
        }

        // --- NEW: Public API functions ---

        pub fn write(self: *@This(), data: []const u8) !void {
            _ = try self.writer.writeAll(data);
        }

        pub fn read(self: *@This()) []const u8 {
            // Returns the unread portion of the reader_buff
            return self.reader.buffered();
        }

        pub fn writer_flush(self: *@This()) void {
            if (self.writer.buffered().len == 0) return;

            if (!self.epin_busy) {
                self.epin_busy = true;
                self.device.?.endpoint_transfer(self.ep_in, self.writer.buffered());
                // Reset the writer buffer
                self.writer = io.Writer.fixed(&self.writer_buff);
            }
        }

        pub fn reader_reset(self: *@This()) void {
            self.reader = io.Reader.fixed(&.{});
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
                self.device.?.endpoint_open(curr_cfg[0..desc_ep.length]);
                endpoints_found += 1;
                curr_cfg = bos.get_desc_next(curr_cfg);
            }

            if (self.ep_in == 0 or self.ep_out == 0 or self.ep_boot == 0) {
                return error.UsbDriverError;
            }

            self.reader = io.Reader.fixed(&.{});
            self.writer = io.Writer.fixed(&self.writer_buff);

            self.epin_busy = false;
            self.epout_primed = false;

            // Prime the OUT endpoint to receive data into our reader_buff
            if (!self.epin_busy and !self.epout_primed) {
                self.epout_primed = true;
                self.device.?.endpoint_transfer(self.ep_out, &self.reader_buff);
            }

            self.device.?.endpoint_transfer(self.ep_boot, &self.boot_buff);

            return start_len - curr_cfg.len;
        }

        /// Called for class/vendor-specific control requests.
        fn class_control(
            _: *anyopaque,
            _: types.ControlStage,
            _: *const types.SetupPacket,
        ) bool {
            return true;
        }

        /// This is the core logic. Called when data is received (OUT) or sent (IN).
        fn transfer(ptr: *anyopaque, ep_addr: u8, data: []u8) void {
            var self: *@This() = @ptrCast(@alignCast(ptr));

            if (ep_addr == self.ep_out) {
                self.epout_primed = false; // It's no longer primed, it's been used.
                self.reader = io.Reader.fixed(data);
            } else if (ep_addr == self.ep_boot) {
                rp2xxx.rom.reset_to_usb_boot();
            } else if (ep_addr == self.ep_in) {
                // IN transfer (our reply) is complete
                self.epin_busy = false;

                // Now that we've sent our reply, we can re-arm the OUT endpoint
                // to receive the *next* command from the host.
                if (!self.epout_primed) {
                    self.epout_primed = true;
                    self.device.?.endpoint_transfer(self.ep_out, &self.reader_buff);
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
