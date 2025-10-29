// usb_config.zig
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const usb = rp2xxx.usb;

// --- User Configuration ---
// All device-specific values are grouped here for easy modification.

const VENDOR_ID: u16 = 0x1E8B;
const PRODUCT_ID: u16 = 0x0001;
const DEVICE_VERSION: u16 = 0x0100; // e.g., 1.0.0

const USB_VERSION: u16 = 0x0200; // USB 2.0
const MAX_POWER_MA: u9 = 100; // Max power draw in mA
const CONFIG_ATTRIBUTES: u8 = 0xC0; // Self-powered, no remote wakeup

// IAD (Interface Association Descriptor) device class for a composite device like CDC
const DEVICE_CLASS_IAD = 0xEF;
const DEVICE_SUBCLASS_IAD = 2;
const DEVICE_PROTOCOL_IAD = 1;

// String descriptors are 1-indexed by the host. Index 0 is the Language ID.
// This enum maps logical names to their 1-based index.
const StringIndex = enum(u8) {
    Manufacturer = 1,
    Product = 2,
    Serial = 3,
    Cdc0Interface = 4,
    Cdc1Interface = 5,
};

// --- Endpoint Configuration ---
const MAX_PACKET_SIZE_0: u8 = 64;
const CDC_EP_NOTIFICATION_SIZE: u16 = 8;
const CDC_EP_DATA_SIZE: u16 = 64;

// --- CDC 0 Configuration ---
// These values are taken directly from the C example
const CDC1_INTERFACE_NUM: u8 = 0;
const CDC1_EP_NOTIFICATION_ADDR: u8 = 0x81; // EP 1 IN
const CDC1_EP_DATA_OUT_ADDR: u8 = 0x01; // EP 1 OUT
const CDC1_EP_DATA_IN_ADDR: u8 = 0x82; // EP 2 IN

// --- CDC 1 Configuration ---
// These values are taken directly from the C example
const CDC0_INTERFACE_NUM: u8 = 2; // CDC 0 uses 0 & 1
const CDC0_EP_NOTIFICATION_ADDR: u8 = 0x83; // EP 3 IN
const CDC0_EP_DATA_OUT_ADDR: u8 = 0x03; // EP 3 OUT
const CDC0_EP_DATA_IN_ADDR: u8 = 0x84; // EP 4 IN

// --- End User Configuration ---

// USB peripheral instance
const usb_dev = rp2xxx.usb.Usb(.{});

// --- String Descriptors ---
const LANG_DESCRIPTOR = "\x04\x03\x09\x04"; // U.S. English
const ALL_DESCRIPTOR_STRINGS = &.{
    &usb.utils.utf8_to_utf16_le("LogicWeave"), // Index 1
    &usb.utils.utf8_to_utf16_le("LogicWeaveCore"), // Index 2
    &usb.utils.utf8_to_utf16_le("someserial"), // Index 3
    &usb.utils.utf8_to_utf16_le("LogicWeave Driver"), // Index 4 (CDC 0)
    &usb.utils.utf8_to_utf16_le("LogicWeave WebUI"), // Index 5 (CDC 1)
};

// --- Combined Configuration Descriptor ---
const usb_config_len = usb.templates.config_descriptor_len + (2 * usb.templates.cdc_descriptor_len);

const usb_config_descriptor =
    // 1. Main Configuration Descriptor
    usb.templates.config_descriptor(
        1, // bConfigurationValue
        4, // bNumInterfaces (2 for each CDC device)
        0, // iConfiguration (0 = no string)
        usb_config_len, // wTotalLength
        CONFIG_ATTRIBUTES, // bmAttributes
        MAX_POWER_MA, // bMaxPower
    ) ++
    // 2. CDC 0 (Serial) Descriptors
    usb.templates.cdc_descriptor(
        CDC0_INTERFACE_NUM, // bFirstInterface
        @intFromEnum(StringIndex.Cdc0Interface), // iInterface
        CDC0_EP_NOTIFICATION_ADDR, // Notification EP Address
        CDC_EP_NOTIFICATION_SIZE, // Notification EP Size
        CDC0_EP_DATA_OUT_ADDR, // Data EP OUT Address
        CDC0_EP_DATA_IN_ADDR, // Data EP IN Address
        CDC_EP_DATA_SIZE, // Data EP Size
    ) ++
    // 3. CDC 1 (Serial) Descriptors
    usb.templates.cdc_descriptor(
        CDC1_INTERFACE_NUM, // bFirstInterface
        @intFromEnum(StringIndex.Cdc1Interface), // iInterface
        CDC1_EP_NOTIFICATION_ADDR, // Notification EP Address
        CDC_EP_NOTIFICATION_SIZE, // Notification EP Size
        CDC1_EP_DATA_OUT_ADDR, // Data EP OUT Address
        CDC1_EP_DATA_IN_ADDR, // Data EP IN Address
        CDC_EP_DATA_SIZE, // Data EP Size
    );

// --- USB Drivers ---
pub var cdc0_driver: usb.cdc.CdcClassDriver(usb_dev) = .{};
pub var cdc1_driver: usb.cdc.CdcClassDriver(usb_dev) = .{};

var drivers = [_]usb.types.UsbClassDriver{ cdc0_driver.driver(), cdc1_driver.driver() };

// --- Top-Level Device Configuration ---
pub var DEVICE_CONFIGURATION: usb.DeviceConfiguration = .{
    .device_descriptor = &.{
        .descriptor_type = usb.DescType.Device,
        .bcd_usb = USB_VERSION,
        .device_class = DEVICE_CLASS_IAD,
        .device_subclass = DEVICE_SUBCLASS_IAD,
        .device_protocol = DEVICE_PROTOCOL_IAD,
        .max_packet_size0 = MAX_PACKET_SIZE_0,
        .vendor = VENDOR_ID,
        .product = PRODUCT_ID,
        .bcd_device = DEVICE_VERSION,
        .manufacturer_s = @intFromEnum(StringIndex.Manufacturer),
        .product_s = @intFromEnum(StringIndex.Product),
        .serial_s = @intFromEnum(StringIndex.Serial),
        .num_configurations = 1,
    },
    .config_descriptor = &usb_config_descriptor,
    .lang_descriptor = LANG_DESCRIPTOR,
    .descriptor_strings = ALL_DESCRIPTOR_STRINGS,
    .drivers = &drivers,
};
