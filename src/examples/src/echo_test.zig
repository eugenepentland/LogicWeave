// logicweave_core.zig
const std = @import("std");
const microzig = @import("microzig");
const EchoVendorDriver = @import("echo_vendor_driver.zig").EchoVendorDriver; // 1. Import new driver

const rp2xxx = microzig.hal;
const flash = rp2xxx.flash;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const usb = rp2xxx.usb;

const led = gpio.num(25);
const uart = rp2xxx.uart.instance.num(0);
const uart_tx_pin = gpio.num(0);
const uart_rx_pin = gpio.num(1);

const usb_dev = rp2xxx.usb.Usb(.{});

// 2. (FIXED) Use the existing vendor descriptor template from your templates.zig
const usb_config_len = usb.templates.config_descriptor_len + usb.templates.vendor_descriptor_len;
const usb_config_descriptor =
    usb.templates.config_descriptor(
        1, // config index
        1, // number of interfaces
        0, // string index
        usb_config_len,
        0xc0, // attributes (self-powered)
        100, // max power (100 * 2mA = 200mA)
    ) ++
    usb.templates.vendor_descriptor(
        0, // interface_number
        0, // string_index
        usb.Endpoint.to_address(2, .Out), // endpoint_out_address
        usb.Endpoint.to_address(1, .In), // endpoint_in_address
        64, // endpoint_size
    );

// 3. Instantiate your new driver
var driver_vendor: EchoVendorDriver(usb_dev) = .{};
// 4. Update the drivers array
var drivers = [_]usb.types.UsbClassDriver{driver_vendor.driver()};

// 5. Update Device Configuration
pub var DEVICE_CONFIGURATION: usb.DeviceConfiguration = .{
    .device_descriptor = &.{
        .descriptor_type = usb.DescType.Device,
        .bcd_usb = 0x0200,
        .device_class = 0xFF, // 0xFF = Vendor Specific
        .device_subclass = 0x00,
        .device_protocol = 0x00,
        .max_packet_size0 = 64,
        .vendor = 0x2E8A, // Raspberry Pi
        .product = 0x000a, // Pico SDK example
        .bcd_device = 0x0100,
        .manufacturer_s = 1,
        .product_s = 2,
        .serial_s = 0,
        .num_configurations = 1,
    },
    .config_descriptor = &usb_config_descriptor,
    .lang_descriptor = "\x04\x03\x09\x04", // length || string descriptor (0x03) || Engl (0x0409)
    .descriptor_strings = &.{
        &usb.utils.utf8_to_utf16_le("MyCompany"),
        &usb.utils.utf8_to_utf16_le("Echo Device"),
        &usb.utils.utf8_to_utf16_le("123456"),
        &usb.utils.utf8_to_utf16_le("Board CDC"),
    },
    .drivers = &drivers,
};

// 6. (FIXED) Corrected panic signature
pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.log.err("panic: {s}", .{message});
    @breakpoint();
    while (true) {}
}

pub fn main() !void {
    // First we initialize the USB clock
    usb_dev.init_clk();
    // Then initialize the USB device using the configuration defined above
    usb_dev.init_device(&DEVICE_CONFIGURATION) catch unreachable;

    // 7. Simplified main loop
    while (true) {
        // The custom driver handles all echo logic within its callbacks.
        // We just need to poll the USB task to process events.
        usb_dev.task(
            false, // debug output over UART [Y/n]
        ) catch unreachable;
    }
}
