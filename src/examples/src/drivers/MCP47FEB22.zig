const std = @import("std");
// Assuming 'microzig' and 'rp2xxx' are available for I2C and HAL access,
// just like in your example.
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const i2c = rp2xxx.i2c;
const I2CMutex = @import("i2c_mutex.zig").I2CMutex;

pub const MCP47FEB22 = @This();

// From the datasheet (Table 6-1), the A2 variant has the I2C address 1100010b (0x62).
const DEVICE_ADDRESS: i2c.Address = @enumFromInt(0x62);

// Volatile DAC register addresses (from Table 4-1 of the datasheet)
pub const Channel = enum(u8) {
    // Note: The Python example used '2' and '1' as keys for channels 0 and 1,
    // which seems non-standard. The datasheet uses address 0x00 for DAC0 and 0x01 for DAC1.
    // We will use standard channel indexing (0 and 1) for clarity,
    // and map them to the correct volatile memory address.
    dac0 = 0x00, // Volatile DAC Register Address for Channel 0
    dac1 = 0x01, // Volatile DAC Register Address for Channel 1

    pub fn asU8(self: Channel) u8 {
        return @intFromEnum(self);
    }
};

i2c_instance: I2CMutex,

/// Initializes the MCP47FEB22 controller.
pub fn init(i2c_instance: I2CMutex) MCP47FEB22 {
    return MCP47FEB22{ .i2c_instance = i2c_instance };
}

/// Deinitializes the device.
pub fn deinit(self: *MCP47FEB22) void {
    // Reset I2C peripheral (or similar cleanup)
    self.i2c_instance.reset();
}

// --- Public API Functions ---

/// Sets the output voltage for a specific DAC channel.
/// The output voltage is V_out = (V_ref * value) / 4096.
///
/// Args:
///     channel: The DAC channel to control (.dac0 or .dac1).
///     value: The 12-bit digital value (0-4095) to set.
pub fn set_voltage(self: *MCP47FEB22, channel: Channel, value: u16) !void {
    // 1. Value validation (0-4095 for 12-bit)
    if (value > 4095) {
        return error.ValueOutOfRange;
    }

    // 2. Determine the memory address
    // This is simply the enum value.
    const mem_address = channel.asU8();

    // 3. Construct the command byte (Figure 7-1 in datasheet)
    // Command Byte = [AD4, AD3, AD2, AD1, AD0, C1, C0, x]
    // C1, C0 = '00' for a write command.
    // AD4..AD0 are the memory address (mem_address).
    // The Python example simplifies this to (mem_address << 3).
    const command_byte: u8 = mem_address << 3;

    // 4. Construct the two data bytes
    // The DAC value is 12 bits, sent in two bytes.
    // Data Byte 1 (Upper 4 bits are ignored/0): [0, 0, 0, 0, D11, D10, D9, D8]
    // Data Byte 2 (Lower 8 bits): [D7, D6, D5, D4, D3, D2, D1, D0]
    const data_byte_1: u8 = @intCast((value >> 8) & 0x0F);
    const data_byte_2: u8 = @intCast(value & 0xFF);

    // 5. Construct the payload
    // The payload is: [Command Byte] [Data Byte 1] [Data Byte 2]
    const payload: [3]u8 = .{ command_byte, data_byte_1, data_byte_2 };

    // 6. Perform the I2C write
    // The Zig I2C example uses write_blocking for commands that are just a write.
    // I2C Write format: [Device Address + W] [Payload...] [Stop]
    try self.i2c_instance.write_blocking(
        DEVICE_ADDRESS,
        &payload,
        null, // No callback/extra data needed
    );
}
