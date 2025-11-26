const std = @import("std");
const microzig = @import("microzig");
const LogicWeaveDriver = @import("logicweave_driver.zig").LogicWeaveDriver;

const rp2xxx = microzig.hal;
const flash = rp2xxx.flash;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const usb = rp2xxx.usb;
const types = rp2xxx.usb.types;

const usb_dev = rp2xxx.usb.Usb(.{});

// --- Interface 1 Endpoints (Driver) ---
const ep_echo_in_addr = usb.Endpoint.to_address(1, .In);
const ep_echo_out_addr = usb.Endpoint.to_address(2, .Out);
const ep_boot_addr = usb.Endpoint.to_address(3, .Out);

// --- Interface 2 Endpoints (Web) ---
// We must use different endpoint numbers for the second interface to avoid collisions
const ep_web_echo_in_addr = usb.Endpoint.to_address(4, .In);
const ep_web_echo_out_addr = usb.Endpoint.to_address(5, .Out);
const ep_web_boot_addr = usb.Endpoint.to_address(6, .Out);

const ep_size = 64;

const interface_desc_len = 9;
const endpoint_desc_len = 7;

// Updated length: Config + (2 * Interface) + (6 * Endpoints)
const usb_config_len = usb.templates.config_descriptor_len + (2 * interface_desc_len) + (6 * endpoint_desc_len);

const MS_VENDOR_CODE: u8 = 0x20;

const msft_string_descriptor = [_]u8{ 18, 0x03, 'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0, MS_VENDOR_CODE, 0x00 };

// --- Interface 1 Descriptor (Driver) ---
const id1 = &types.InterfaceDescriptor{
    .interface_number = 0, // Index 0
    .alternate_setting = 0,
    .num_endpoints = 3,
    .interface_class = 0xFF,
    .interface_subclass = 0,
    .interface_protocol = 0,
    .interface_s = 0,
};

// --- Interface 1 Endpoints ---
const ed1_in = &types.EndpointDescriptor{
    .endpoint_address = ep_echo_in_addr,
    .attributes = @intFromEnum(types.TransferType.Bulk),
    .max_packet_size = ep_size,
    .interval = 0,
};

const ed1_out = &types.EndpointDescriptor{
    .endpoint_address = ep_echo_out_addr,
    .attributes = @intFromEnum(types.TransferType.Bulk),
    .max_packet_size = ep_size,
    .interval = 0,
};

const ed1_boot = &types.EndpointDescriptor{
    .endpoint_address = ep_boot_addr,
    .attributes = @intFromEnum(types.TransferType.Bulk),
    .max_packet_size = ep_size,
    .interval = 0,
};

// --- Interface 2 Descriptor (Web) ---
const id2 = &types.InterfaceDescriptor{
    .interface_number = 1, // Index 1
    .alternate_setting = 0,
    .num_endpoints = 3,
    .interface_class = 0xFF,
    .interface_subclass = 0,
    .interface_protocol = 0,
    .interface_s = 0,
};

// --- Interface 2 Endpoints ---
const ed2_in = &types.EndpointDescriptor{
    .endpoint_address = ep_web_echo_in_addr,
    .attributes = @intFromEnum(types.TransferType.Bulk),
    .max_packet_size = ep_size,
    .interval = 0,
};

const ed2_out = &types.EndpointDescriptor{
    .endpoint_address = ep_web_echo_out_addr,
    .attributes = @intFromEnum(types.TransferType.Bulk),
    .max_packet_size = ep_size,
    .interval = 0,
};

const ed2_boot = &types.EndpointDescriptor{
    .endpoint_address = ep_web_boot_addr,
    .attributes = @intFromEnum(types.TransferType.Bulk),
    .max_packet_size = ep_size,
    .interval = 0,
};

// UUID for WebUSB
const WEBUSB_GUID = [_]u8{
    0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47,
    0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
};

const WEBUSB_VENDOR_CODE: u8 = 0x30;
const WEBUSB_LANDING_PAGE_INDEX: u8 = 1;

const webusb_platform_capability_descriptor = [_]u8{
    0x18, // bLength
    0x10, // bDescriptorType = DEVICE CAPABILITY
    0x05, // bDevCapabilityType = PLATFORM
    0x00, // bReserved
    // PlatformCapabilityUUID
    WEBUSB_GUID[0], WEBUSB_GUID[1], WEBUSB_GUID[2], WEBUSB_GUID[3],
    WEBUSB_GUID[4], WEBUSB_GUID[5], WEBUSB_GUID[6], WEBUSB_GUID[7],
    WEBUSB_GUID[8], WEBUSB_GUID[9], WEBUSB_GUID[10], WEBUSB_GUID[11],
    WEBUSB_GUID[12], WEBUSB_GUID[13], WEBUSB_GUID[14], WEBUSB_GUID[15],
    0x00, 0x01, // bcdVersion 1.0
    WEBUSB_VENDOR_CODE,
    WEBUSB_LANDING_PAGE_INDEX,
};

const full_bos_descriptor = [_]u8{
    0x05, // bLength
    0x0F, // bDescriptorType = BOS
    (5 + 24), 0x00, // wTotalLength
    0x01, // bNumDeviceCaps
} ++ webusb_platform_capability_descriptor;

const usb_config_descriptor =
    usb.templates.config_descriptor(
        1, // config index
        2, // number of interfaces (Updated to 2)
        0, // string index
        usb_config_len,
        0xc0, // attributes
        100, // max power
    ) ++ 
    // Interface 1 (Driver)
    id1.serialize() ++
    ed1_in.serialize() ++
    ed1_out.serialize() ++
    ed1_boot.serialize() ++
    // Interface 2 (Web)
    id2.serialize() ++
    ed2_in.serialize() ++
    ed2_out.serialize() ++
    ed2_boot.serialize();

// --- Instantiating Two Drivers ---
pub var driver: LogicWeaveDriver(usb_dev) = .{};
pub var web: LogicWeaveDriver(usb_dev) = .{};

// Add both to the driver array
pub var drivers = [_]usb.types.UsbClassDriver{
    driver.driver(), 
    web.driver()
};

const WEBUSB_URL = "logicweave.eugenepentland.dev/";

const webusb_url_descriptor = &[_]u8{
    @as(u8, WEBUSB_URL.len + 3),
    0x03,
    0x01,
} ++ WEBUSB_URL;

pub var DEVICE_CONFIGURATION: usb.DeviceConfiguration = .{
    .device_descriptor = &.{
        .descriptor_type = usb.DescType.Device,
        .bcd_usb = 0x0210,
        .device_class = 0xFF,
        .device_subclass = 0x00,
        .device_protocol = 0x00,
        .max_packet_size0 = 64,
        .vendor = 0x2E8A,
        .product = 0x000a,
        .bcd_device = 0x0100,
        .manufacturer_s = 1,
        .product_s = 2,
        .serial_s = 0,
        .num_configurations = 1,
    },
    .bos_descriptor = &full_bos_descriptor,
    .config_descriptor = &usb_config_descriptor,
    .webusb_vendor_code = WEBUSB_VENDOR_CODE,
    .webusb_landing_page_index = WEBUSB_LANDING_PAGE_INDEX,
    .webusb_url_descriptor = webusb_url_descriptor,
    .lang_descriptor = "\x04\x03\x09\x04",
    .descriptor_strings = &.{
        &usb.utils.utf8_to_utf16_le("LogicWeave"),
        &usb.utils.utf8_to_utf16_le("LogicWeave Core"),
        &usb.utils.utf8_to_utf16_le("123456"),
        &usb.utils.utf8_to_utf16_le("Board"),
        &msft_string_descriptor,
    },
    .drivers = &drivers,
};