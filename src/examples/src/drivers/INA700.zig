const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const i2c = rp2xxx.i2c;

pub const INA700 = @This();

// --- Configuration ---

// Register Addresses (from INA700 Datasheet)
pub const Register = enum(u8) {
    CONFIG = 0x00,
    VBUS = 0x05,
    DIETEMP = 0x06,
    CURRENT = 0x07,
    POWER = 0x08,
    ENERGY = 0x09,
    CHARGE = 0x0A,
    MANUFACTURER_ID = 0x3E,

    pub fn asU8(self: Register) u8 {
        return @intFromEnum(self);
    }
};

// LSB conversion factors. Using floating point for calculations.
// The raw values are typically stored as u32/u64 for the math.
// Note: We use f64 for better precision.
pub const VOLTAGE_LSB: f64 = 0.003125; // 3.125 mV/LSB
pub const CURRENT_LSB: f64 = 0.000480; // 480 µA/LSB
pub const TEMPERATURE_LSB: f64 = 0.125; // 125 m°C/LSB
pub const POWER_LSB: f64 = 0.000096; // 96 µW/LSB
pub const ENERGY_LSB: f64 = 0.001536; // 1.536 mJ/LSB
pub const CHARGE_LSB: f64 = 0.000030; // 30 µC/LSB

// Default I2C address
pub const DEFAULT_ADDRESS: i2c.Address = @enumFromInt(0x44);

// --- Struct Definition ---

i2c_instance: i2c.I2C,
address: i2c.Address,

pub fn init(i2c_instance: i2c.I2C, address: i2c.Address) INA700 {
    return INA700{ .i2c_instance = i2c_instance, .address = address };
}

pub fn deinit(self: *INA700) void {
    self.i2c_instance.reset();
}

// --- Private I2C Helper Functions ---

/// Reads a raw, multi-byte value from a specified register.
/// The device is Big Endian (MSB first) for data registers.
fn readRegister(self: *INA700, reg_addr: Register, byte_count: usize) !u64 {
    // 1. Prepare the register address byte to send (writev part)
    const reg_byte = reg_addr.asU8();

    // 2. Prepare the buffer to read into (readv part)
    // We use a maximum size of 5 bytes (for ENERGY/CHARGE 40-bit)
    var read_buffer: [5]u8 = undefined;
    if (byte_count == 0 or byte_count > 5) {
        return error.InvalidByteCount;
    }
    const read_slice = read_buffer[0..byte_count];

    // I2C sequence: [START] + [DevAddr+W] + [RegAddr] + [REPEATED START] + [DevAddr+R] + [Read Data...] + [STOP]
    try self.i2c_instance.writev_then_readv_blocking(
        self.address,
        &.{&.{reg_byte}}, // Slice of slices for the write part (just the register address)
        &.{read_slice}, // Slice of slices for the read part (the destination buffer)
        microzig.drivers.time.Duration.from_ms(100), // Timeout
    );

    // 3. Convert the Big-Endian bytes into a u64 integer
    // We need to shift and OR the bytes manually since std.mem.readInt only handles native types (u16, u32, u64).
    var raw_value: u64 = 0;
    for (read_slice, 0..) |byte, i| {
        // The most significant byte is read first (index 0).
        // It needs to be shifted by (byte_count - 1 - i) * 8 bits.
        const shift = (byte_count - 1 - i) * 8;
        raw_value |= @as(u64, byte) << @as(u6, @intCast(shift));
    }

    return raw_value;
}

/// Writes a 16-bit value to a specified register.
fn writeRegister16bit(self: *INA700, reg_addr: Register, value: u16) !void {
    // Write payload: [Register Address] [Data MSB] [Data LSB]
    // The device uses Big-Endian for the data write.
    const payload: [3]u8 = .{
        reg_addr.asU8(),
        @as(u8, value >> 8), // Data MSB
        @as(u8, value & 0xFF), // Data LSB
    };

    // I2C Write format: [Device Address + W] [Payload...] [Stop]
    try self.i2c_instance.write_blocking(
        self.address,
        &payload,
        null,
    );
}

// --- Public API Functions (Commands) ---

/// Triggers a software reset of the INA700.
pub fn reset(self: *INA700) !void {
    // Setting bit 15 of the CONFIG register to 1 triggers a reset (0x8000).
    try self.writeRegister16bit(.CONFIG, 0x8000);
}

/// Resets the ENERGY and CHARGE accumulation registers to 0.
pub fn resetAccumulators(self: *INA700) !void {
    // Setting bit 14 of the CONFIG register to 1 resets accumulators (0x4000).
    try self.writeRegister16bit(.CONFIG, 0x4000);
}

/// Reads the manufacturer ID (0x5449 for "TI").
pub fn readManufacturerId(self: *INA700) !u16 {
    // The manufacturer ID is 2 bytes (16-bit)
    const raw_id = try self.readRegister(.MANUFACTURER_ID, 2);
    return @intCast(raw_id);
}

// --- Public API Functions (Measurements) ---

/// Reads the bus voltage.
/// Returns: Voltage in Volts (V).
pub fn readVoltage(self: *INA700) !f32 {
    // VBUS is 2 bytes (16-bit) and unsigned.
    const raw_voltage = try self.readRegister(.VBUS, 2);

    // Convert the raw u16 value to f32.
    const raw_voltage_f32: f32 = @floatFromInt(raw_voltage);

    // Explicitly cast the VOLTAGE_LSB constant (assuming it's f64) to f32.
    const lsb_f32: f32 = @floatCast(VOLTAGE_LSB);

    // V_out = Raw * LSB. The multiplication is now performed using f32 types.
    return raw_voltage_f32 * lsb_f32;
}

/// Reads the internal die temperature.
/// Returns: Temperature in degrees Celsius (°C).
pub fn readTemperature(self: *INA700) !f64 {
    // DIETEMP is 2 bytes (16-bit) but contains a 12-bit two's complement value (bits 15-4)
    const raw_temp = try self.readRegister(.DIETEMP, 2);

    // 1. Shift right by 4 to align the 12-bit value.
    const temp_val_u16: u16 = @intCast(raw_temp >> 4);

    // 2. Perform 12-bit two's complement conversion.
    // Max positive value is 2^11 - 1 = 2047 (0x7FF).
    // If the 12th bit (bit 11 in the shifted value) is set, it's negative.
    if (temp_val_u16 & (1 << 11) != 0) {
        // Subtract 2^12 (4096) to get the negative value.
        // We use i16 for the math to handle the signed conversion easily.
        var temp_val_i16: i16 = @intCast(temp_val_u16);
        temp_val_i16 -= (1 << 12);

        // 3. Apply LSB and return
        return @as(f64, @floatFromInt(temp_val_i16)) * TEMPERATURE_LSB;
    }

    // If positive or zero:
    return @as(f64, temp_val_u16) * TEMPERATURE_LSB;
}

/// Reads the current.
/// Returns: Current magnitude in Amperes (A).
pub fn readCurrent(self: *INA700) !f32 {
    // CURRENT is 2 bytes (16-bit) and uses two's complement.
    const raw_current = try self.readRegister(.CURRENT, 2);

    // 1. Perform 16-bit two's complement conversion.
    const current_val_u16: u16 = @intCast(raw_current);
    const signed_raw_val: i16 = @as(i16, @bitCast(current_val_u16));

    // 2. Apply LSB and return the absolute value (magnitude).

    // Convert the raw i16 value to f32.
    const raw_current_f32: f32 = @floatFromInt(signed_raw_val);

    // Explicitly cast the f64 LSB constant to f32 using @floatCast.
    const lsb_f32: f32 = @floatCast(CURRENT_LSB);

    // Perform multiplication using f32 types.
    const current_f32 = raw_current_f32 * lsb_f32;

    return @abs(current_f32);
}

/// Reads the calculated power.
/// Returns: Power in Watts (W).
pub fn readPower(self: *INA700) !f64 {
    // POWER is 3 bytes (24-bit) and unsigned.
    const raw_power = try self.readRegister(.POWER, 3);
    // V_out = Raw * LSB
    return @as(f64, raw_power) * POWER_LSB;
}

/// Reads the accumulated energy.
/// Returns: Energy in Joules (J).
pub fn readEnergy(self: *INA700) !f64 {
    // ENERGY is 5 bytes (40-bit) and unsigned.
    // We read it into a u64 which is large enough.
    const raw_energy = try self.readRegister(.ENERGY, 5);
    return @as(f64, raw_energy) * ENERGY_LSB;
}

/// Reads the accumulated charge.
/// Returns: Charge in Coulombs (C).
pub fn readCharge(self: *INA700) !f64 {
    // CHARGE is 5 bytes (40-bit) and uses two's complement.
    const raw_charge = try self.readRegister(.CHARGE, 5);

    // 1. Perform 40-bit two's complement conversion.
    // Since Zig's largest integer is u64/i64, we need to check the 40th bit (bit 39).
    var charge_val_i64: i64 = @intCast(raw_charge);

    if (raw_charge & (1 << 39) != 0) {
        // If the sign bit (bit 39) is set, we need to subtract 2^40.
        // We can do this by setting all the upper bits (40-63) to 1.
        // The mask for the upper 24 bits (64 - 40) is `0xFFFFFF0000000000`
        charge_val_i64 |= ~((1 << 40) - 1);
    }

    // 2. Apply LSB and return
    return @as(f64, @floatFromInt(charge_val_i64)) * CHARGE_LSB;
}
