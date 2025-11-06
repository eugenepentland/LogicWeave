// echo_vendor_driver.zig
const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;

const types = microzig.core.usb.types;
const utils = microzig.core.usb.utils;

const bos = utils.BosConfig;
const DescType = types.DescType;

pub fn EchoVendorDriver(comptime usb: anytype) type {
    _ = usb; // (FIXED) Mark parameter as intentionally unused
    return struct {
        device: ?types.UsbDevice = null,
        ep_in: u8 = 0,
        ep_out: u8 = 0,

        /// Called by the USB stack during device initialization
        fn init(ptr: *anyopaque, device: types.UsbDevice) void {
            var self: *@This() = @ptrCast(@alignCast(ptr));
            self.device = device;
        }

        /// Called by the USB stack when the host sets the configuration.
        /// We parse our descriptors, find our endpoints, and open them.
        fn open(ptr: *anyopaque, cfg: []const u8) !usize {
            var self: *@This() = @ptrCast(@alignCast(ptr));
            var curr_cfg = cfg;
            const start_len = cfg.len;

            // 1. Expect Interface Descriptor
            if (bos.try_get_desc_as(types.InterfaceDescriptor, curr_cfg)) |desc_itf| {
                if (desc_itf.interface_class != 0xFF) { // 0xFF = Vendor Specific
                    return types.DriverErrors.UnsupportedInterfaceClassType;
                }
            } else {
                return types.DriverErrors.ExpectedInterfaceDescriptor;
            }
            curr_cfg = bos.get_desc_next(curr_cfg);

            // 2. Expect Endpoints (we expect 2: one IN, one OUT)
            var endpoints_found: u8 = 0;
            while (endpoints_found < 2 and curr_cfg.len > 0 and bos.get_desc_type(curr_cfg) == DescType.Endpoint) {
                const desc_ep = bos.get_desc_as(types.EndpointDescriptor, curr_cfg);

                switch (types.Endpoint.dir_from_address(desc_ep.endpoint_address)) {
                    .In => self.ep_in = desc_ep.endpoint_address,
                    .Out => self.ep_out = desc_ep.endpoint_address,
                }

                // Tell the core to open this endpoint
                self.device.?.endpoint_open(curr_cfg[0..desc_ep.length]);

                endpoints_found += 1;
                curr_cfg = bos.get_desc_next(curr_cfg);
            }

            if (self.ep_in == 0 or self.ep_out == 0) {
                // We didn't find both our endpoints
                return error.UsbDriverError;
            }

            // Prime the OUT endpoint to be ready to receive data
            self.device.?.endpoint_transfer(self.ep_out, &.{});

            return start_len - curr_cfg.len; // Return total bytes parsed
        }

        /// Called for class/vendor-specific control requests. We don't have any.
        fn class_control(
            ptr: *anyopaque,
            stage: types.ControlStage,
            setup: *const types.SetupPacket,
        ) bool {
            _ = ptr;
            _ = stage;
            _ = setup;
            // Return true to indicate we handled it (by ignoring it)
            return true;
        }

        /// This is the core logic. Called when data is received (OUT) or sent (IN).
        fn transfer(ptr: *anyopaque, ep_addr: u8, data: []u8) void {
            var self: *@This() = @ptrCast(@alignCast(ptr));

            if (ep_addr == self.ep_out) {
                // Data received from host on our OUT endpoint

                // Echo it back on the IN endpoint
                self.device.?.endpoint_transfer(self.ep_in, data);

                // (FIX) DO NOT re-prime the OUT endpoint here.
                // We will wait for the IN transfer to complete first.
                // self.device.?.endpoint_transfer(self.ep_out, &.{});
            }

            // add another endpoint only to force rebooting into the bootloader
            // rp2xxx.rom.reset_to_usb_boot()

            if (ep_addr == self.ep_in) {
                // Data has finished sending to the host.

                // (FIX) NOW we can re-prime the OUT endpoint,
                // making us ready to receive the *next* packet.
                self.device.?.endpoint_transfer(self.ep_out, &.{});
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
