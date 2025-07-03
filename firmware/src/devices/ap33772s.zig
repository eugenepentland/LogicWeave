const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const i2c = rp2xxx.i2c;
const gpio = rp2xxx.gpio;

const AP33772S = @This();

// I2C Address for the AP33772S (Datasheet Page 16) [cite: 178]
const DEVICE_ADDRESS: rp2xxx.i2c.Address = @enumFromInt(0x52);
var readBuff: [2]u8 = undefined;

// Register Addresses (Commands) from Datasheet Table 16 [cite: 174]
pub const Register = enum(u8) {
    STATUS = 0x01,
    MASK = 0x02,
    OPMODE = 0x03,
    CONFIG = 0x04,
    PDCONFIG = 0x05,
    SYSTEM = 0x06,
    TR25 = 0x0C,
    TR50 = 0x0D,
    TR75 = 0x0E,
    TR100 = 0x0F,
    VOLTAGE = 0x11, // VOUT Voltage, LSB 80mV
    CURRENT = 0x12, // VOUT Current, LSB 24mA
    TEMP = 0x13, // Temperature, Unit: °C, Offset? (+25°C default?) - Needs verification
    VREQ = 0x14, // Requested Voltage, LSB 50mV
    IREQ = 0x15, // Requested Current, LSB 10mA
    VSELMIN = 0x16, // Min Selection Voltage, LSB 200mV
    UVPTHR = 0x17, // UVP threshold %
    OVPTHR = 0x18, // OVP threshold offset, LSB 80mV
    OCPTHR = 0x19, // OCP threshold, LSB 50mA
    OTPTHR = 0x1A, // OTP threshold, Unit: °C
    DRTHR = 0x1B, // De-rating threshold, Unit: °C
    SRCPDO = 0x20, // Get All Source PDOs (26 bytes)
    SRC_SPR_PDO1 = 0x21,
    SRC_SPR_PDO2 = 0x22,
    SRC_SPR_PDO3 = 0x23,
    SRC_SPR_PDO4 = 0x24,
    SRC_SPR_PDO5 = 0x25,
    SRC_SPR_PDO6 = 0x26,
    SRC_SPR_PDO7 = 0x27,
    SRC_EPR_PDO8 = 0x28,
    SRC_EPR_PDO9 = 0x29,
    SRC_EPR_PD010 = 0x2A,
    SRC_EPR_PDO11 = 0x2B,
    SRC_EPR_PDO12 = 0x2C,
    SRC_EPR_PDO13 = 0x2D,
    PD_REQMSG = 0x31, // Send PDO Request (2 bytes)
    PD_CMDMSG = 0x32, // Send specific PD command
    PD_MSGRLT = 0x33, // Result of PD request/command

    // Helper to convert enum to u8
    pub fn asU8(self: Register) u8 {
        return @intFromEnum(self);
    }
};

i2c_instance: i2c.I2C,
gate_open: bool = false,
gate_gpio: rp2xxx.gpio.Pin,

pub fn init(sda_pin: u8, scl_pin: u8, gate_pin: u8, i2c_instance: i2c.I2C) !AP33772S {
    const sda_gpio = rp2xxx.gpio.num(sda_pin);
    const scl_gpio = rp2xxx.gpio.num(scl_pin);

    inline for (&.{ scl_gpio, sda_gpio }) |pin| {
        pin.set_slew_rate(.slow);
        pin.set_schmitt_trigger(.enabled);
        pin.set_function(.i2c);
    }

    try i2c_instance.apply(.{
        .clock_config = rp2xxx.clock_config,
    });

    const gate_gpio = rp2xxx.gpio.num(gate_pin);

    gate_gpio.set_function(.sio);
    gate_gpio.set_direction(.out);

    var device = AP33772S{ .i2c_instance = i2c_instance, .gate_gpio = gate_gpio, .gate_open = false };
    device.enable(false);
    device.configureProtections(DEFAULT_CONFIG) catch {};
    return device;
}

pub fn enable(self: *@This(), state: bool) void {
    self.gate_open = state;
    self.gate_gpio.put(@intFromBool(!state));
}

pub fn toggle(self: *@This()) void {
    self.enable(!self.gate_open);
}

pub fn deinit(self: *AP33772S) void {
    // Reset I2C peripheral
    self.i2c_instance.reset();
    std.log.info("AP33772S Deinitialized\n", .{});
}

// --- Private Helper Functions ---

// Writes data to a specific register/command
fn writeRegister(self: *AP33772S, reg_addr: Register, data: []const u8) !void {
    // I2C Write format: [Device Address + W] [Command Byte] [Data Bytes...] [Stop] [cite: 179, 182, 183]
    // Need a buffer for command + data
    var write_buffer: [32]u8 = undefined; // Adjust size as needed, max 26 for SRCPDO? Check other commands. PD_REQMSG needs 3 bytes total.
    if (data.len + 1 > write_buffer.len) {
        std.log.err("Write buffer too small for reg {x} data len {}\n", .{ reg_addr.asU8(), data.len });
        return error.BufferTooSmall;
    }

    write_buffer[0] = reg_addr.asU8();
    @memcpy(write_buffer[1..][0..data.len], data);

    try self.i2c_instance.write_blocking(DEVICE_ADDRESS, write_buffer[0 .. data.len + 1], null);
    // No explicit delay needed here unless datasheet specifies for a particular command
}

// Reads data from a specific register/command
fn readRegister(self: *AP33772S, reg_addr: Register, buffer: []u8) !void {
    // I2C Read format: [Device Address + W] [Command Byte] [Restart] [Device Address + R] [Read Data Bytes...] [Stop]
    const reg_byte = reg_addr.asU8();

    // Use writev_then_readv_blocking for the standard register read pattern:
    // [START] + [DevAddr+W] + [RegAddr] + [REPEATED START] + [DevAddr+R] + [Read Data...] + [STOP]
    // The first slice contains the byte(s) to write (just the register address here)
    // The second slice contains the buffer(s) to read into
    try self.i2c_instance.writev_then_readv_blocking(DEVICE_ADDRESS, &.{&.{reg_byte}}, &.{buffer}, microzig.drivers.time.Duration.from_ms(100));
}
// --- Public API Functions ---

const Status = packed struct {
    _reserved: u1,
    over_temperature_protection: u1,
    over_current_protection: u1,
    over_voltage_protection: u1,
    under_voltage_protection: u1,
    new_pdo: u1,
    ready: u1,
    started: u1,
};

const Mask = packed struct {
    _reserved: u1,
    OTP: u1,
    OCP: u1,
    OVP: u1,
    NEWPDO: u1,
    READY: u1,
    STARTED: u1,
};

const Config = packed struct {
    DR_EN: u1,
    OTP_EN: u1,
    OCP_EN: u1,
    OVP_EN: u1,
    UVP_EN: u1,
    _reserved1: u1,
    _reserved2: u1,
    _reserved3: u1,
};

pub const DEFAULT_CONFIG: Config = .{
    ._reserved1 = 0,
    ._reserved2 = 0,
    ._reserved3 = 0,
    .DR_EN = 0,
    .OCP_EN = 1,
    .OTP_EN = 1,
    .OVP_EN = 1,
    .UVP_EN = 1,
};

pub fn getStatus(self: *AP33772S) !Status {
    var status_byte = [1]u8{0};
    try self.readRegister(.STATUS, status_byte[0..]);
    return @as(Status, @bitCast(status_byte[0]));
}

pub fn getOperationMode(self: *AP33772S) !u8 {
    var opmode_byte: u8 = undefined;
    try self.readRegister(.OPMODE, &opmode_byte);
    return opmode_byte;
}

// Reads the VOUT voltage register (0x11) and converts to millivolts [cite: 174]
pub fn readVoltageMv(self: *AP33772S) !u16 {
    try self.readRegister(.VOLTAGE, &readBuff);
    // Data is Little Endian (Datasheet Page 16) [cite: 185]
    const raw_voltage = std.mem.readInt(u16, readBuff[0..], .little);
    return raw_voltage * 80; // LSB is 80mV [cite: 174]
}

// Reads the VOUT current register (0x12) and converts to milliamps
pub fn readCurrentMa(self: *AP33772S) !u16 {
    try self.readRegister(.CURRENT, readBuff[0..]); // Pass a mutable slice
    return @as(u16, readBuff[0]) * 24; // LSB is 24mA
}

// Reads the TEMP register (0x13) [cite: 174]
// Note: Datasheet indicates Unit: °C, but default is 19h (+25°C).
// Requires initialization of TRxx registers for accuracy. [cite: 167]
// Consider if an offset needs to be applied based on TRxx calibration.
pub fn readTemperature(self: *AP33772S) !u8 {
    try self.readRegister(.TEMP, readBuff[0..]);
    return readBuff[0];
}

// Reads the requested voltage (VREQ) register (0x14) and converts to millivolts [cite: 174]
pub fn readRequestedVoltageMv(self: *AP33772S) !u16 {
    var vreq_bytes: [2]u8 = undefined;
    try self.readRegister(.VREQ, &vreq_bytes);
    const raw_vreq = std.mem.readInt(u16, vreq_bytes[0..], .little);
    return raw_vreq * 50; // LSB is 50mV [cite: 174]
}

// Reads the requested current (IREQ) register (0x15) and converts to milliamps [cite: 174]
pub fn readRequestedCurrentMa(self: *AP33772S) !u16 {
    var ireq_bytes: [2]u8 = undefined;
    try self.readRegister(.IREQ, &ireq_bytes);
    const raw_ireq = std.mem.readInt(u16, ireq_bytes[0..], .little);
    return raw_ireq * 10; // LSB is 10mA [cite: 174]
}

pub const PDORequest = packed struct {
    pdo_index: u4,
    current_select: u4,
    voltage_select: u8,
};

// Sends a PDO request message (PD_REQMSG 0x31) [cite: 82, 174]
// pdo_request format (16-bit):
// Bits 12:15: PDO_INDEX (1-7 for SPR, 8-13 for EPR)
// Bits 8:11: CURRENT_SEL (0-15 for 1.00A - 5.00A, TBD exact mapping)
// Bits 0:7: VOLTAGE_SEL (mV/100 for PPS, mV/200 for AVS)
pub fn requestPDO(self: *AP33772S, pdo_request: PDORequest) !void {
    var request_bytes: [2]u8 = undefined;
    std.mem.writeInt(u16, request_bytes[0..], @bitCast(pdo_request), .little); // Little Endian [cite: 185]
    try self.writeRegister(.PD_REQMSG, &request_bytes);
    // May need to check PD_MSGRLT (0x33) afterwards for success/failure [cite: 84]
}

// Reads the result of the last PD request/command (PD_MSGRLT 0x33) [cite: 174]
pub fn getPdMessageResult(self: *AP33772S) !u8 {
    try self.readRegister(.PD_MSGRLT, readBuff[0..]);
    return readBuff[0];
}

// Example: Set OTP Threshold (OTPTHR 0x1A) [cite: 98, 174]
pub fn setOTPThreshold(self: *AP33772S, threshold_celsius: u8) !void {
    try self.writeRegister(.OTPTHR, &.{threshold_celsius});
}

pub fn readMask(self: *AP33772S) !Mask {
    try self.readRegister(.MASK, readBuff[0..]);
    return @bitCast(readBuff[0]);
}

// Example: Configure protection features (CONFIG 0x04) [cite: 137, 174]
pub fn configureProtections(self: *AP33772S, config_byte: Config) !void {
    try self.writeRegister(.CONFIG, &.{@bitCast(config_byte)});
}

pub fn setPDConfig(self: *AP33772S, config_byte: u8) !void {
    try self.writeRegister(.PDCONFIG, &.{config_byte});
}

// Example: Read all source PDOs (SRCPDO 0x20, 26 bytes) [cite: 75, 174]
pub fn readAllSourcePDOs(self: *AP33772S, pdo_buffer: []u8) !void {
    if (pdo_buffer.len < 26) {
        std.log.err("Buffer too small for reading all PDOs (need 26 bytes)\n", .{});
        return error.BufferTooSmall;
    }
    try self.readRegister(.SRCPDO, pdo_buffer[0..26]);
}

pub fn readSourcePDO(self: *AP33772S, index: u8) !SRC_PDO {
    // Validate index: PDO indices are typically 1-based, corresponding to registers 0x21-0x2D
    if (index < 1 or index > 13) {
        std.log.err("Invalid PDO index: {}. Must be between 1 and 13.", .{index});
        return error.InvalidIndex;
    }

    // Calculate the register address (0x20 + index)
    const register_addr_u8 = 0x20 + index;

    // Convert u8 to Register enum - this is safe because we checked the range 1..13
    // which maps to 0x21..0x2D, all valid Register enum values.
    const register: Register = @enumFromInt(register_addr_u8);

    // Buffer to read the 2-byte PDO
    var pdo_bytes: [2]u8 = undefined;

    // Read 2 bytes from the specific PDO register
    try self.readRegister(register, pdo_bytes[0..]);

    // Read the type to know if its fixed,

    // Cast the 2 bytes into the union type
    // std.mem.readInt is typically for reading into a primitive type like u16,
    // but @bitCast works directly with the raw bytes array [2]u8
    // assuming the memory layout matches the packed union definition.
    // Since the device is Little Endian, the [2]u8 array correctly maps byte 0 to the low bits
    // and byte 1 to the high bits, matching the packed union definition.
    const pdo = @as(SRC_PDO, @bitCast(pdo_bytes));

    return pdo;
}

const VoltageMin = enum(u2) {
    // These names/values are illustrative based on your partial code
    // and the display functions. You might want more descriptive names.
    Reserved = 0,
    MinThreshold1 = 1, // Corresponds to 3300mV~ (SPR) or 15000mV~ (EPR)
    MinThreshold2 = 2, // Corresponds to 3300-5000mV (SPR) or 15000-20000mV (EPR)
    Others = 3,
};

pub const SRC_PDO = packed struct {
    voltage_max: u8, // Bits 7:0
    peak_current_or_voltage_min: u2, // Bits 9:8
    current_max_code: u4, // Bits 13:10 - Renamed for clarity
    type: u1, // Bit 14 (should be 0 for fixed, 1 for PPS/AVS)
    detect: u1, // Bit 15

    // Helper to get the voltage value in mV (scaling depends on type and EPR)
    // Note: This function needs to know if it's an EPR PDO for correct scaling
    pub fn get_voltage_mv(self: @This(), is_epr: bool) u32 {
        // For Fixed PDOs (type == 0), scaling is 100mV/LSB for SPR, 200mV/LSB for EPR
        // For PPS PDOs (type == 1), scaling is 100mV/LSB
        // For AVS PDOs (type == 1), scaling is 200mV/LSB.
        const scale_mv: u32 = if (is_epr) 200 else 100;
        return @as(u32, @intCast(self.voltage_max)) * scale_mv;
    }

    pub fn get_current_ma(self: @This()) u32 {
        return 1000 + (@as(u32, @intCast(self.current_max_code)) * 266);
    }

    pub fn is_pps(self: @This()) bool {
        return self.type == 1;
    }

    pub fn get_voltage_min_mv(self: @This(), is_epr: bool) u32 {
        const voltage_min_code: VoltageMin = @enumFromInt(self.peak_current_or_voltage_min);
        return switch (voltage_min_code) {
            .Reserved => 0, // Or handle as an error/unknown state
            .MinThreshold1 => if (is_epr) 15000 else 3300,
            .MinThreshold2 => if (is_epr) 15000 else 5000, // Lower bound of the range
            .Others => 0, // Needs clarification from datasheet if specific
        };
    }
};
