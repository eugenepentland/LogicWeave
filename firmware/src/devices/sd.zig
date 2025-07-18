const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;

// Define custom errors for SD card operations
pub const SDCardError = error{
    Cmd0Failed,
    Cmd8Failed,
    Acmd41Timeout,
    Cmd55Failed,
    Cmd58Failed,
    Cmd16Failed,
    Cmd9Failed,
    Cmd10Failed,
    Cmd17Failed,
    Cmd24Failed,
    NoDataToken,
    DataNotAccepted,
    CardBusyTimeout,
    InvalidDataLength,
    // Add more specific errors as needed based on SD card specification
};

pub const SD_Driver = @This();

allocator: std.mem.Allocator,
cs_pin: gpio.Pin,
mosi_pin: gpio.Pin, // Using PioPin as it's common for SPI in RP2040
miso_pin: gpio.Pin,
sclk_pin: gpio.Pin,
spi: rp2xxx.spi.SPI, // Store the configured SPI instance
is_sdhc: bool, // Flag to indicate if the card is SDHC/SDXC

// SD Card Command Definitions (as per SD Physical Layer Simplified Specification)
const CMD0 = 0x40; // GO_IDLE_STATE (Software reset)
const CMD8 = 0x48; // SEND_IF_COND (For SDHC/SDXC detection and voltage check)
const CMD9 = 0x49; // SEND_CSD (Read Card-Specific Data register)
const CMD10 = 0x4A; // SEND_CID (Read Card Identification register)
const CMD16 = 0x50; // SET_BLOCKLEN (Set block length for SDSC cards)
const CMD17 = 0x51; // READ_SINGLE_BLOCK (Read a single data block)
const CMD24 = 0x58; // WRITE_BLOCK (Write a single data block)
const CMD55 = 0x77; // APP_CMD (Prefix for application-specific commands)
const CMD58 = 0x7A; // READ_OCR (Read Operating Conditions Register)
const ACMD41 = 0x69; // SD_SEND_OP_COND (Initialize SD card, preceded by CMD55)

/// Initializes the SD_Driver struct and configures the necessary GPIO pins and SPI peripheral.
///
/// Parameters:
///   alloc: The allocator to be used for dynamic memory operations within the driver.
///   cs_p: The Chip Select (CS) GPIO pin.
///   sclk_p: The Serial Clock (SCLK) GPIO pin.
///   mosi_p: The Master Out Slave In (MOSI) GPIO pin.
///   miso_p: The Master In Slave Out (MISO) GPIO pin.
///   spi_inst: The SPI peripheral instance (e.g., `rp2xxx.spi.instance.SPI0`).
///   initial_baud_rate: The baud rate to use during the initial card discovery phase (e.g., 100_000 Hz).
///
/// Returns:
///   An initialized `SD_Driver` instance or an error.
pub fn init(
    alloc: std.mem.Allocator,
    cs_p: gpio.Pin,
    sclk_p: gpio.Pin,
    mosi_p: gpio.Pin,
    miso_p: gpio.Pin,
    spi_inst: rp2xxx.spi.SPI,
    initial_baud_rate: u32,
) !SD_Driver {
    // Configure CS Pin for SIO (Software Input/Output) and set as output
    cs_p.set_function(.sio);
    cs_p.set_direction(.out);
    cs_p.put(1); // Start with CS HIGH (card deselected)

    // Configure SCLK, MOSI, MISO pins for SPI function
    sclk_p.set_function(.spi);
    mosi_p.set_function(.spi);
    miso_p.set_function(.spi);

    // Configure the passed SPI instance with the initial baud rate
    var configured_spi = spi_inst;
    try configured_spi.set_baudrate(initial_baud_rate, rp2xxx.clock_config.peri.?.frequency());

    // Create and return the SD_Driver instance
    const driver = SD_Driver{
        .allocator = alloc,
        .cs_pin = cs_p,
        .sclk_pin = sclk_p,
        .mosi_pin = mosi_p,
        .miso_pin = miso_p,
        .spi = configured_spi,
        .is_sdhc = false, // This flag will be determined during the initialization process
    };

    return driver;
}

/// Sets the Chip Select (CS) pin low to select the SD card.
/// Includes a small delay for stability.
fn _cs_low(self: *SD_Driver) void {
    self.cs_pin.put(0);
    time.sleep_us(500); // 0.5ms delay, adjust if needed for specific cards/setups
}

/// Sets the Chip Select (CS) pin high to deselect the SD card.
/// Includes a small delay for stability.
fn _cs_high(self: *SD_Driver) void {
    self.cs_pin.put(1);
    time.sleep_us(1000); // 1ms delay, allows card to finish internal operations
}

/// Sends a command packet to the SD card via SPI.
/// A command packet consists of: Command byte, 4-byte argument, and 1-byte CRC.
///
/// Parameters:
///   cmd: The command byte (e.g., `CMD0`, `CMD8`).
///   arg: The 32-bit argument for the command.
///   crc: The 7-bit CRC checksum for the command packet (plus MSB always 1).
fn _send_command(self: *SD_Driver, cmd: u8, arg: u32, crc: u8) void {
    var packet: [6]u8 = undefined;
    packet[0] = cmd;
    packet[1] = @intCast((arg >> 24) & 0xFF);
    packet[2] = @intCast((arg >> 16) & 0xFF);
    packet[3] = @intCast((arg >> 8) & 0xFF);
    packet[4] = @intCast(arg & 0xFF);
    packet[5] = crc; // CRC is 7 bits, but sent as an 8-bit byte with MSB always 1

    self.spi.write_blocking(u8, &packet);
}

/// Reads a response from the SD card. It polls for a non-0xFF byte (R1 response)
/// and then reads additional bytes if `length` is greater than 1.
///
/// Parameters:
///   length: The total number of bytes expected in the response (e.g., 1 for R1, 5 for R3/R7).
///   retries: The maximum number of times to poll for the initial response byte.
///
/// Returns:
///   A dynamically allocated slice containing the response bytes, or an `SDCardError.NoDataToken` error.
///   The caller is responsible for freeing this memory using `self.allocator.free()`.
fn _read_response(self: *SD_Driver, length: usize, retries: usize) ![]u8 {
    var response_buffer = try self.allocator.alloc(u8, length);
    // If an error occurs after allocation, ensure the buffer is freed.
    errdefer self.allocator.free(response_buffer);

    var i: usize = 0;
    while (i < retries) {
        var byte_read: [1]u8 = undefined;
        // Read one byte, sending 0xFF as dummy data to clock out the response
        _ = self.spi.read_blocking(u8, 0xFF, byte_read[0..]);
        if (byte_read[0] != 0xFF) { // Check if a valid response (not 0xFF) is received
            response_buffer[0] = byte_read[0];
            // If more bytes are expected, read them
            if (length > 1) {
                for (0..length - 1) |j| {
                    _ = self.spi.read_blocking(u8, 0xFF, byte_read[0..]);
                    response_buffer[j] = byte_read[0];
                }
            }
            return response_buffer; // Return the allocated buffer
        }
        time.sleep_us(1000); // 1ms delay between retries
        i += 1;
    }
    // If we reach here, no valid response was received. Free the buffer and return error.
    self.allocator.free(response_buffer);
    return SDCardError.NoDataToken;
}

/// Waits for a data token (0xFE) from the SD card, indicating the start of a data block.
///
/// Parameters:
///   retries: The maximum number of times to poll for the data token.
///
/// Returns:
///   `void` on success, or an `SDCardError.NoDataToken` error if the token is not received within retries.
fn _wait_for_data_token(self: *SD_Driver, retries: usize) !void {
    var i: usize = 0;
    while (i < retries) {
        var token: u8 = 0xFF;
        _ = self.spi.read_blocking(u8, &[_]u8{0xFF}, &[_]u8{&token});
        if (token == 0xFE) { // Data Start Token
            return;
        }
        i += 1;
    }
    return SDCardError.NoDataToken; // Data token not received
}

/// Executes the full SD card initialization sequence.
/// This brings the SD card from a power-on state to a data-ready state.
///
/// Parameters:
///   high_baud_rate: The desired high baud rate for data transfer after initialization.
///
/// Returns:
///   `void` on successful initialization, or an `SDCardError` if any step fails.
pub fn initialize(self: *SD_Driver, high_baud_rate: u32) !void {
    // 1. Send 74+ clock cycles (10 * 8 = 80 clock cycles) with CS high.
    // This provides the card with enough clock pulses to enter SPI mode.
    var dummy_bytes: [10]u8 = [_]u8{0xFF} ** 10;
    self.spi.write_blocking(u8, &dummy_bytes);

    // 2. CMD0: GO_IDLE_STATE
    // Puts the card into idle state. Expected R1 response: 0x01.
    self._cs_low();
    const r1_cmd0 = try self._read_response(1, 20);
    defer self.allocator.free(r1_cmd0); // Free allocated memory
    self._send_command(CMD0, 0x00000000, 0x95); // CRC for CMD0 with 0 arg is 0x95
    self._cs_high();
    if (r1_cmd0[0] != 0x01) {
        return SDCardError.Cmd0Failed;
    }

    // 3. CMD8: SEND_IF_COND (for SDHC/SDXC detection and voltage check)
    // Checks if the card supports 2.7-3.6V and returns a check pattern.
    // Argument: 0x000001AA (VHS: 0x1 (2.7-3.6V), Check Pattern: 0xAA)
    // Expected R7 response (R1 + 4 data bytes).
    self._cs_low();
    const r7_cmd8 = try self._read_response(5, 20);
    defer self.allocator.free(r7_cmd8); // Free allocated memory
    self._send_command(CMD8, 0x000001AA, 0x87); // CRC for CMD8 with 0x1AA arg is 0x87
    self._cs_high();
    if (r7_cmd8[0] == 0x01 and r7_cmd8[1] == 0x00 and r7_cmd8[2] == 0x00 and r7_cmd8[3] == 0x01 and r7_cmd8[4] == 0xAA) {
        self.is_sdhc = true; // Card supports CMD8 and voltage, likely SDHC/SDXC
    } else if (r7_cmd8[0] == 0x05) { // Illegal Command (CMD8 not supported, likely SDSC)
        self.is_sdhc = false;
    } else {
        return SDCardError.Cmd8Failed;
    }

    // 4. ACMD41: SD_SEND_OP_COND (initialization process)
    // This command is preceded by CMD55 (APP_CMD). It's used to bring the card out of idle state.
    // Argument: HCS bit (bit 30) set for SDHC/SDXC, otherwise 0.
    const acmd41_arg: u32 = if (self.is_sdhc) 0x40000000 else 0x00000000; // HCS bit for SDHC/SDXC
    var i: usize = 0;
    while (i < 100) { // Max 100 retries (approx 5 seconds total)
        // Send CMD55 (APP_CMD)
        self._cs_low();
        const r1_cmd55 = try self._read_response(1, 20);
        defer self.allocator.free(r1_cmd55); // Free allocated memory
        self._send_command(CMD55, 0x00000000, 0x65); // CRC for CMD55 is 0x65
        if (r1_cmd55[0] > 0x01) { // R1 response should be 0x00 or 0x01
            self._cs_high();
            return SDCardError.Cmd55Failed;
        }

        // Send ACMD41 (SD_SEND_OP_COND)
        const r1_acmd41 = try self._read_response(1, 20);
        defer self.allocator.free(r1_acmd41); // Free allocated memory
        self._send_command(ACMD41, acmd41_arg, 0x77); // CRC for ACMD41 is 0x77
        self._cs_high();
        if (r1_acmd41[0] == 0x00) { // R1 response 0x00 indicates card is initialized
            break; // Initialization complete
        }
        time.sleep_us(50000); // 50ms delay before next retry
        i += 1;
    }
    if (i == 100) {
        return SDCardError.Acmd41Timeout;
    }

    // 5. If SDHC/SDXC, read OCR (CMD58) to confirm CCS (Card Capacity Status) bit.
    // This bit indicates if the card is Standard Capacity (0) or High/Extended Capacity (1).
    if (self.is_sdhc) {
        self._cs_low();
        const r3_cmd58 = try self._read_response(5, 20); // R3 response: R1 + 4 data bytes (OCR)
        defer self.allocator.free(r3_cmd58); // Free allocated memory
        self._send_command(CMD58, 0x00000000, 0xFD); // READ_OCR, CRC is 0xFD
        self._cs_high();
        if (r3_cmd58[0] != 0x00) {
            return SDCardError.Cmd58Failed;
        }
        // Check CCS bit (bit 30 of OCR, which is byte 1, bit 6 of R3 response)
        self.is_sdhc = (r3_cmd58[1] & 0x40) != 0;
    }

    // 6. If SDSC, set block length to 512 bytes (CMD16).
    // SDHC/SDXC cards always use 512-byte blocks, so this command is not needed for them.
    if (!self.is_sdhc) {
        self._cs_low();
        const r1_cmd16 = try self._read_response(1, 20);
        defer self.allocator.free(r1_cmd16); // Free allocated memory
        self._send_command(CMD16, 512, 0x15); // SET_BLOCKLEN to 512 bytes, CRC is 0x15
        self._cs_high();
        if (r1_cmd16[0] != 0x00) {
            return SDCardError.Cmd16Failed;
        }
    }

    // 7. Increase SPI baud rate for faster data transfer.
    try self.spi.set_baudrate(high_baud_rate, rp2xxx.clock_config.peri.?.frequency());
}

/// Reads the 16-byte Card-Specific Data (CSD) register from the SD card.
///
/// Returns:
///   A 16-byte array containing the raw CSD data, or an `SDCardError` on failure.
pub fn read_csd(self: *SD_Driver) ![16]u8 {
    self._cs_low();
    self._send_command(CMD9, 0x00000000, 0xAF); // SEND_CSD, CRC is 0xAF
    const r1 = try self._read_response(1, 20);
    defer self.allocator.free(r1); // Free allocated memory
    if (r1[0] != 0x00) {
        self._cs_high();
        return SDCardError.Cmd9Failed;
    }

    try self._wait_for_data_token(1000); // Wait for data token (0xFE)

    // Read 16 bytes of CSD data + 2 bytes of CRC16
    var csd_data_and_crc: [18]u8 = undefined;
    var dummy_read_data: [18]u8 = [_]u8{0xFF} ** 18; // Dummy bytes to send during read
    _ = self.spi.read_blocking(u8, &dummy_read_data, &csd_data_and_crc);
    self._cs_high();

    var csd_raw: [16]u8 = undefined;
    @memcpy(&csd_raw, csd_data_and_crc[0..16]); // Copy only the 16 CSD bytes
    return csd_raw;
}

/// Reads the 16-byte Card Identification (CID) register from the SD card.
///
/// Returns:
///   A 16-byte array containing the raw CID data, or an `SDCardError` on failure.
pub fn read_cid(self: *SD_Driver) ![16]u8 {
    self._cs_low();
    self._send_command(CMD10, 0x00000000, 0xEF); // SEND_CID, CRC is 0xEF
    const r1 = try self._read_response(1, 20);
    defer self.allocator.free(r1); // Free allocated memory
    if (r1[0] != 0x00) {
        self._cs_high();
        return SDCardError.Cmd10Failed;
    }

    try self._wait_for_data_token(1000); // Wait for data token (0xFE)

    // Read 16 bytes of CID data + 2 bytes of CRC16
    var cid_data_and_crc: [18]u8 = undefined;
    var dummy_read_data: [18]u8 = [_]u8{0xFF} ** 18; // Dummy bytes to send during read
    _ = self.spi.read_blocking(u8, &dummy_read_data, &cid_data_and_crc);
    self._cs_high();

    var cid_raw: [16]u8 = undefined;
    @memcpy(&cid_raw, cid_data_and_crc[0..16]); // Copy only the 16 CID bytes
    return cid_raw;
}

/// Reads a single data block from the SD card.
///
/// Parameters:
///   block_address: The address of the block to read. For SDHC/SDXC, this is the block number.
///                  For SDSC, it's also the block number (assuming 512-byte blocks due to CMD16).
///   block_size: The expected size of the block in bytes (e.g., 512).
///
/// Returns:
///   A mutable slice of bytes containing the data read from the block, or an `SDCardError` on failure.
///   The caller is responsible for freeing the allocated memory for the returned slice.
pub fn read_block(self: *SD_Driver, block_address: u32, block_size: usize) ![]u8 {
    if (block_size == 0) {
        return SDCardError.InvalidDataLength; // Prevent zero-length allocation
    }

    self._cs_low();
    // CMD17 argument is the block address.
    // For SDHC/SDXC, this is directly the block number.
    // For SDSC, if CMD16 has set block length to 512, it's also the block number.
    self._send_command(CMD17, block_address, 0x29); // READ_SINGLE_BLOCK, CRC is 0x29
    const r1 = try self._read_response(1, 20);
    defer self.allocator.free(r1); // Free allocated memory
    if (r1[0] != 0x00) {
        self._cs_high();
        return SDCardError.Cmd17Failed;
    }

    try self._wait_for_data_token(1000); // Wait for data token (0xFE)

    // Allocate buffer for block_size bytes + 2 bytes for CRC16
    var data_and_crc_buffer = try self.allocator.alloc(u8, block_size + 2);
    defer self.allocator.free(data_and_crc_buffer); // Ensure this is freed on error

    // Create a dummy buffer of 0xFFs to send during the read operation
    const dummy_read_data = try self.allocator.alloc(u8, block_size + 2);
    defer self.allocator.free(dummy_read_data); // Ensure this is freed on error
    @memset(dummy_read_data, 0xFF); // Fill with 0xFF

    // Perform the SPI read
    _ = self.spi.read_blocking(u8, dummy_read_data, data_and_crc_buffer);
    self._cs_high();

    // Allocate a new slice for just the data (without CRC)
    const data_slice = try self.allocator.alloc(u8, block_size);
    @memcpy(data_slice, data_and_crc_buffer[0..block_size]); // Copy only the data part
    return data_slice; // Return the data slice, caller must free it
}

/// Writes a single data block to the SD card.
///
/// Parameters:
///   block_address: The address of the block to write.
///   data: The data to write (must be 512 bytes for standard blocks).
///
/// Returns:
///   `void` on successful write, or an `SDCardError` on failure.
pub fn write_block(self: *SD_Driver, block_address: u32, data: []const u8) !void {
    // For standard SD cards, block size is typically 512 bytes.
    if (data.len != 512) {
        return SDCardError.InvalidDataLength;
    }

    self._cs_low();
    self._send_command(CMD24, block_address, 0x8B); // WRITE_BLOCK, CRC is 0x8B
    const r1 = try self._read_response(1, 20);
    defer self.allocator.free(r1); // Free allocated memory
    if (r1[0] != 0x00) {
        self._cs_high();
        return SDCardError.Cmd24Failed;
    }

    // Send data start token (Single Block Write)
    self.spi.write_blocking(u8, &[_]u8{0xFE});
    // Send data block
    self.spi.write_blocking(u8, data);
    // Send dummy CRC16 (0xFFFF) - actual CRC can be calculated for robustness
    self.spi.write_blocking(u8, &[_]u8{ 0xFF, 0xFF });

    // Read data response token
    const data_response = try self._read_response(1, 20);
    defer self.allocator.free(data_response); // Free allocated memory
    // Check for Data Accepted (0x05 is the lower 3 bits)
    if ((data_response[0] & 0x1F) != 0x05) {
        self._cs_high();
        return SDCardError.DataNotAccepted;
    }

    // Wait for busy signal to clear (MISO goes high)
    var i: usize = 0;
    while (i < 5000) { // ~500ms timeout for busy (5000 * 0.1ms)
        var busy_status: u8 = 0;
        _ = self.spi.read_blocking(u8, &[_]u8{0xFF}, &[_]u8{&busy_status});
        if (busy_status == 0xFF) { // Card is no longer busy
            break;
        }
        time.sleep_us(100); // 0.1ms delay for busy polling
        i += 1;
    }
    if (i == 5000) {
        self._cs_high();
        return SDCardError.CardBusyTimeout;
    }

    self._cs_high();
}