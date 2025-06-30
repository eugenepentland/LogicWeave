const std = @import("std");
const microzig = @import("microzig");
const rp2040 = microzig.hal;
const time = rp2040.time;
const i2c = rp2040.i2c;
const gpio = rp2040.gpio;

const EEPROM24C02 = @This();

i2c_instance: i2c.I2C,
wc_pin: gpio.Pin,
config: i2c.Config,

// Enum to represent EEPROM addresses
pub const Address = enum(u7) {
    EEPROM_0 = 0x50,
    EEPROM_1 = 0x51,
    EEPROM_2 = 0x52,
    EEPROM_3 = 0x53,
    EEPROM_4 = 0x54,
    EEPROM_5 = 0x55,
    EEPROM_6 = 0x56,
    EEPROM_7 = 0x57,
};

pub fn init(i2c_instance: i2c.I2C, wc_pin_num: u6, config: i2c.Config) EEPROM24C02 {
    var eeprom = EEPROM24C02{
        .i2c_instance = i2c_instance,
        .wc_pin = gpio.num(wc_pin_num),
        .config = config,
    };

    // Configure the WC pin as output and set it low (write enabled)
    eeprom.wc_pin.set_function(.sio);
    eeprom.wc_pin.set_direction(.out);
    eeprom.wc_pin.put(0);

    // Initialize I2C
    i2c_instance.apply(config) catch |err| {
        std.log.err("Failed to initialize I2C: {}\n", .{err});
        // Handle the error appropriately, maybe halt or return an error
    };

    return eeprom;
}

pub fn deinit(self: *EEPROM24C02) void {
    // Reset I2C peripheral
    self.i2c_instance.reset();

    // Reset WC pin to a safe state (e.g., input with pull-up)
    self.wc_pin.set_function(.sio);
    self.wc_pin.set_direction(.in);
    self.wc_pin.set_pull(.up);
}

pub fn writeByte(self: *EEPROM24C02, eeprom_address: Address, address: u8, data: u8) !void {
    // Enable write
    self.wc_pin.put(0);

    // Send data to be written
    var command = [_]u8{ address, data };
    try self.i2c_instance.write_blocking(@intFromEnum(eeprom_address), &command, null);

    // Wait for write cycle to complete (datasheet specifies 5ms max)
    time.sleep_ms(5);
}

pub fn writePage(self: *EEPROM24C02, eeprom_address: Address, address: u8, data: []const u8) !void {
    // Ensure data doesn't exceed page boundary (16 bytes)
    if (data.len > 16 or (address & 0xF0) != ((address + @as(u8, @intCast(data.len - 1))) & 0xF0)) {
        return error.PageBoundaryExceeded;
    }

    // Enable write
    self.wc_pin.put(0);

    // Create a buffer with address followed by data
    var buffer: [17]u8 = undefined;
    buffer[0] = address;
    @memcpy(buffer[1..][0..data.len], data);

    // Send data to be written
    try self.i2c_instance.write_blocking(@intFromEnum(eeprom_address), buffer[0 .. data.len + 1], null);

    // Wait for write cycle to complete (datasheet specifies 5ms max)
    time.sleep_ms(5);
}

pub fn readBytes(self: *EEPROM24C02, eeprom_address: Address, address: u8, buffer: []u8) !void {
    // Set read address
    try self.i2c_instance.write_blocking(@intFromEnum(eeprom_address), &.{address}, null);

    // Read data
    try self.i2c_instance.read_blocking(@intFromEnum(eeprom_address), buffer, null);
}

pub fn setWriteProtect(self: *EEPROM24C02, enable: bool) void {
    self.wc_pin.put(if (enable) 1 else 0);
}
