const std = @import("std");
const microzig = @import("microzig");
const LogicWeaveDriver = @import("echo_driver.zig").LogicWeaveDriver;

const rp2xxx = microzig.hal;
const flash = rp2xxx.flash;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const usb = rp2xxx.usb;
const types = rp2xxx.usb.types; // 1. Import types

const usb_dev = rp2xxx.usb.Usb(.{});

const ep_echo_in_addr = usb.Endpoint.to_address(1, .In);
const ep_echo_out_addr = usb.Endpoint.to_address(2, .Out);
const ep_boot_addr = usb.Endpoint.to_address(3, .Out);
const ep_size = 64;

const interface_desc_len = 9;
const endpoint_desc_len = 7;

const usb_config_len = usb.templates.config_descriptor_len + interface_desc_len + (3 * endpoint_desc_len);

const MS_VENDOR_CODE: u8 = 0x20; // Arbitrary non-zero vendor request code

const msft_string_descriptor = [_]u8{ 18, 0x03, 'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0, MS_VENDOR_CODE, 0x00 };

const winusb_compat_id_descriptor = [_]u8{
    0x28, 0x00, 0x00, 0x00, // dwLength
    0x00, 0x01, // bcdVersion 1.0
    0x04, 0x00, // wIndex = Extended Compat ID Descriptor
    0x01, // one function
    0x00, 0x00, 0x00, 0x00, // reserved
    0x00, // interface number
    0x01, // reserved
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0, // Compatible ID
    0, 0, 0, 0, 0, 0, 0, 0, // Sub-Compatible ID
    0, 0, 0, 0, 0, 0, // reserved
};

const id1 = // --- Manually defined Interface Descriptor (9 bytes) ---
    &types.InterfaceDescriptor{ // <--- FIXED (added &)
        .interface_number = 0,
        .alternate_setting = 0,
        .num_endpoints = 3, // We now have 3 endpoints
        .interface_class = 0xFF, // Vendor
        .interface_subclass = 0, // <--- FIXED (field name)
        .interface_protocol = 0, // <--- FIXED (field name)
        .interface_s = 0,
    };

const ed1 = // --- Manually defined Endpoint 1 (Echo IN) (7 bytes) ---
    &types.EndpointDescriptor{ // <--- FIXED (added &)
        .endpoint_address = ep_echo_in_addr,
        .attributes = @intFromEnum(types.TransferType.Bulk),
        .max_packet_size = ep_size,
        .interval = 0,
    };

const ed2 = // --- Manually defined Endpoint 2 (Echo OUT) (7 bytes) ---
    &types.EndpointDescriptor{ // <--- FIXED (added &)
        .endpoint_address = ep_echo_out_addr,
        .attributes = @intFromEnum(types.TransferType.Bulk),
        .max_packet_size = ep_size,
        .interval = 0,
    };

// UUID for WebUSB
const WEBUSB_GUID = [_]u8{
    0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47,
    0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
};

const WEBUSB_VENDOR_CODE: u8 = 0x30; // Match the vendor driver
const WEBUSB_LANDING_PAGE_INDEX: u8 = 1;

// WebUSB platform capability descriptor
const webusb_platform_capability_descriptor = [_]u8{
    0x18, // bLength
    0x10, // bDescriptorType = DEVICE CAPABILITY
    0x05, // bDevCapabilityType = PLATFORM
    0x00, // bReserved
    // PlatformCapabilityUUID (16 bytes)
    WEBUSB_GUID[0],
    WEBUSB_GUID[1],
    WEBUSB_GUID[2],
    WEBUSB_GUID[3],
    WEBUSB_GUID[4],
    WEBUSB_GUID[5],
    WEBUSB_GUID[6],
    WEBUSB_GUID[7],
    WEBUSB_GUID[8],
    WEBUSB_GUID[9],
    WEBUSB_GUID[10],
    WEBUSB_GUID[11],
    WEBUSB_GUID[12],
    WEBUSB_GUID[13],
    WEBUSB_GUID[14],
    WEBUSB_GUID[15],
    0x00, 0x01, // bcdVersion 1.0
    WEBUSB_VENDOR_CODE, // bVendorCode (MUST match your driver's check)
    WEBUSB_LANDING_PAGE_INDEX, // iLandingPage (index of the URL)
};

const full_bos_descriptor = [_]u8{
    // 1. BOS Descriptor Header (5 bytes)
    0x05, // bLength
    0x0F, // bDescriptorType = BOS
    (5 + 24), 0x00, // wTotalLength (5 + 24 = 29 bytes) <-- **CHECK THIS VALUE**
    0x01, // bNumDeviceCaps
    // 2. WebUSB Platform Capability Descriptor (24 bytes)
} ++ webusb_platform_capability_descriptor;

const ed3 = // --- Manually defined Endpoint 3 (Boot OUT) (7 bytes) ---
    &types.EndpointDescriptor{ // <--- FIXED (added &)
        .endpoint_address = ep_boot_addr,
        .attributes = @intFromEnum(types.TransferType.Bulk),
        .max_packet_size = ep_size,
        .interval = 0,
    };

// 3. (FIX) Corrected field names 'interface_subclass' and 'interface_protocol'
const usb_config_descriptor =
    usb.templates.config_descriptor(
        1, // config index
        1, // number of interfaces
        0, // string index
        usb_config_len,
        0xc0, // attributes (self-powered)
        100, // max power (100 * 2mA = 200mA)
    ) ++ id1.serialize() ++
    ed1.serialize() ++
    ed2.serialize() ++
    ed3.serialize(); // <--- Semicolon on the last one

var lw_driver: LogicWeaveDriver(usb_dev) = .{};
var drivers = [_]usb.types.UsbClassDriver{lw_driver.driver()};

const WEBUSB_URL = "webusb.eugenepentland.dev/";

const webusb_url_descriptor = &[_]u8{
    @as(u8, WEBUSB_URL.len + 3), // bLength
    0x03, // URL descriptor
    0x01, // https://
} ++ WEBUSB_URL;

pub var DEVICE_CONFIGURATION: usb.DeviceConfiguration = .{
    .device_descriptor = &.{
        .descriptor_type = usb.DescType.Device,
        .bcd_usb = 0x0210, // USB 2.1 device
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
    .bos_descriptor = &full_bos_descriptor,
    .config_descriptor = &usb_config_descriptor,
    .webusb_vendor_code = WEBUSB_VENDOR_CODE, // 0x30
    .webusb_landing_page_index = WEBUSB_LANDING_PAGE_INDEX, // 1
    .webusb_url_descriptor = webusb_url_descriptor,
    .lang_descriptor = "\x04\x03\x09\x04",
    .descriptor_strings = &.{
        &usb.utils.utf8_to_utf16_le("LogicWeave"),
        &usb.utils.utf8_to_utf16_le("LogicWeave Core"),
        &usb.utils.utf8_to_utf16_le("123456"),
        &usb.utils.utf8_to_utf16_le("Board CDC"),
        &msft_string_descriptor,
    },
    .drivers = &drivers,
};

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.log.err("panic: {s}", .{message});
    @breakpoint();
    while (true) {}
}

pub const microzig_options = microzig.Options{
    .log_level = .debug,
    .logFn = rp2xxx.uart.log,
};

const uart = rp2xxx.uart.instance.num(0);
const uart_tx_pin = gpio.num(12);

pub fn main() !void {
    uart_tx_pin.set_function(.uart);

    uart.apply(.{
        .clock_config = rp2xxx.clock_config,
    });

    rp2xxx.uart.init_logger(uart);

    usb_dev.init_clk();
    usb_dev.init_device(&DEVICE_CONFIGURATION) catch unreachable;

    while (true) {
        usb_dev.task(
            true,
        ) catch unreachable;
        const rx_data = lw_driver.read();
        if (rx_data.len > 0) {
            try lw_driver.write(rx_data);
            lw_driver.writer_flush();

            lw_driver.reader_reset();
        }
    }
}
