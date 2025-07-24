const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const fatfs = @import("zfat");

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
var response_buffer: [8]u8 = undefined;

allocator: std.mem.Allocator,
cs_pin: gpio.Pin,
mosi_pin: gpio.Pin, // Using PioPin as it's common for SPI in RP2040
miso_pin: gpio.Pin,
sclk_pin: gpio.Pin,
spi: rp2xxx.spi.SPI, // Store the configured SPI instance
is_sdhc: bool, // Flag to indicate if the card is SDHC/SDXC
sector_count: u32 = 0, // Add this field
disk: Disk,

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
    // Configure the passed SPI instance with the initial baud rate
    var configured_spi = spi_inst;
    configured_spi.reset();

    try configured_spi.set_baudrate(initial_baud_rate, rp2xxx.clock_config.peri.?.frequency());

    // Configure CS Pin for SIO (Software Input/Output) and set as output
    cs_p.set_function(.sio);
    cs_p.set_direction(.out);
    cs_p.put(1); // Start with CS HIGH (card deselected)

    // Configure SCLK, MOSI, MISO pins for SPI function
    sclk_p.set_function(.spi);
    mosi_p.set_function(.spi);
    miso_p.set_function(.spi);

    try configured_spi.apply(.{ .clock_config = rp2xxx.clock_config });
    time.sleep_us(100);

    // Create and return the SD_Driver instance
    var driver = SD_Driver{
        .allocator = alloc,
        .cs_pin = cs_p,
        .sclk_pin = sclk_p,
        .mosi_pin = mosi_p,
        .miso_pin = miso_p,
        .spi = configured_spi,
        .is_sdhc = false, // This flag will be determined during the initialization process
        .disk = Disk{ .driver = undefined }, // we'll fill this next
    };

    // after you've initialized the rest:
    driver.disk.driver = &driver;

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
    std.log.info("Sending SD CMD {any}", .{cmd});
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
    if (length > 8) {
        return error.InvalidDataLength;
    }

    var i: usize = 0;
    while (i < retries) {
        // Read one byte, sending 0xFF as dummy data to clock out the response
        self.spi.read_blocking(u8, 0xFF, response_buffer[0..1]);
        if (response_buffer[0] != 0xFF) { // Check if a valid response (not 0xFF) is received
            // If more bytes are expected, read them
            if (length > 1) {
                self.spi.read_blocking(u8, 0xFF, response_buffer[1..]);
            }
            return response_buffer[0..length]; // Return the allocated buffer
        }
        i += 1;
    }
    // If we reach here, no valid response was received. Free the buffer and return error.
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
    var token: [1]u8 = undefined;
    while (i < retries) {
        self.spi.read_blocking(u8, 0xFF, &token);
        if (token[0] == 0xFE) { // Data Start Token
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

    time.sleep_ms(100);
    // 2. CMD0: GO_IDLE_STATE
    // Puts the card into idle state. Expected R1 response: 0x01.
    self._cs_low();
    self._send_command(CMD0, 0x00000000, 0x95); // CRC for CMD0 with 0 arg is 0x95
    const r1_cmd0 = try self._read_response(1, 20);
    self._cs_high();
    if (r1_cmd0[0] != 0x01) {
        return SDCardError.Cmd0Failed;
    }

    std.log.info("Put into idle state", .{});

    // 3. CMD8: SEND_IF_COND (for SDHC/SDXC detection and voltage check)
    // Checks if the card supports 2.7-3.6V and returns a check pattern.
    // Argument: 0x000001AA (VHS: 0x1 (2.7-3.6V), Check Pattern: 0xAA)
    // Expected R7 response (R1 + 4 data bytes).
    self._cs_low();
    self._send_command(CMD8, 0x000001AA, 0x87); // CRC for CMD8 with 0x1AA arg is 0x87
    const r7_cmd8 = try self._read_response(5, 20);
    self._cs_high();
    if (r7_cmd8[0] == 0x01 and r7_cmd8[1] == 0x00 and r7_cmd8[2] == 0x00 and r7_cmd8[3] == 0x01 and r7_cmd8[4] == 0xAA) {
        self.is_sdhc = true; // Card supports CMD8 and voltage, likely SDHC/SDXC
    } else if (r7_cmd8[0] == 0x05) { // Illegal Command (CMD8 not supported, likely SDSC)
        self.is_sdhc = false;
    } else {
        return SDCardError.Cmd8Failed;
    }

    std.log.info("Done with cmd8", .{});

    // 4. ACMD41: SD_SEND_OP_COND (initialization process)
    // This command is preceded by CMD55 (APP_CMD). It's used to bring the card out of idle state.
    // Argument: HCS bit (bit 30) set for SDHC/SDXC, otherwise 0.
    const acmd41_arg: u32 = if (self.is_sdhc) 0x40000000 else 0x00000000; // HCS bit for SDHC/SDXC
    var i: usize = 0;
    while (i < 100) { // Max 100 retries (approx 5 seconds total)
        // Send CMD55 (APP_CMD)
        self._cs_low();
        self._send_command(CMD55, 0x00000000, 0x65); // CRC for CMD55 is 0x65
        const r1_cmd55 = try self._read_response(1, 20);
        if (r1_cmd55[0] > 0x01) { // R1 response should be 0x00 or 0x01
            self._cs_high();
            return SDCardError.Cmd55Failed;
        }

        // Send ACMD41 (SD_SEND_OP_COND)
        self._send_command(ACMD41, acmd41_arg, 0x77); // CRC for ACMD41 is 0x77
        const r1_acmd41 = try self._read_response(1, 20);
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
        self._send_command(CMD58, 0x00000000, 0xFD); // READ_OCR, CRC is 0xFD
        const r3_cmd58 = try self._read_response(5, 20); // R3 response: R1 + 4 data bytes (OCR)
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
        self._send_command(CMD16, 512, 0x15); // SET_BLOCKLEN to 512 bytes, CRC is 0x15
        const r1_cmd16 = try self._read_response(1, 20);
        self._cs_high();
        if (r1_cmd16[0] != 0x00) {
            return SDCardError.Cmd16Failed;
        }
    }

    // 7. Read CSD and calculate sector count
    const csd = try self.read_csd();
    // CSD_STRUCTURE is bits 127:126 of the CSD register
    const csd_structure = (csd[0] >> 6) & 0b11;

    if (self.is_sdhc and csd_structure == 1) { // CSD Version 2.0 for SDHC/SDXC
        // C_SIZE is bits 69:48
        const c_size = (@as(u32, csd[7] & 0x3F) << 16) | (@as(u32, csd[8]) << 8) | @as(u32, csd[9]);
        self.sector_count = (c_size + 1) * 1024;
    } else if (!self.is_sdhc and csd_structure == 0) { // CSD Version 1.0 for SDSC
        // READ_BL_LEN is bits 83:80
        const read_bl_len = csd[5] & 0x0F;
        // C_SIZE is bits 73:62
        const c_size = (@as(u32, csd[6] & 0x03) << 10) | (@as(u32, csd[7]) << 2) | (@as(u32, csd[8] >> 6));
        // C_SIZE_MULT is bits 49:47
        const c_size_mult = (@as(u32, (csd[9] & 0x03)) << 1) | @as(u32, csd[10] >> 7);

        const block_len: u64 = @as(u64, 1) << @as(u6, @intCast(read_bl_len));
        const mult: u64 = @as(u64, 1) << (@as(u6, @intCast(c_size_mult)) + 2);
        const block_nr: u64 = c_size + 1;
        const capacity_bytes: u64 = block_nr * mult * block_len;
        self.sector_count = @intCast(capacity_bytes / 512);
    } else {
        // Unsupported CSD version or mismatch
        return SDCardError.Cmd9Failed;
    }
    std.log.info("SD card detected with {d} sectors.", .{self.sector_count});

    // 8. Increase SPI baud rate for faster data transfer.
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
    if (r1[0] != 0x00) {
        self._cs_high();
        return SDCardError.Cmd9Failed;
    }

    try self._wait_for_data_token(1000); // Wait for data token (0xFE)

    // Read 16 bytes of CSD data + 2 bytes of CRC16
    var csd_data_and_crc: [18]u8 = undefined;
    _ = self.spi.read_blocking(u8, 0xFF, &csd_data_and_crc);
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
    if (r1[0] != 0x00) {
        self._cs_high();
        return SDCardError.Cmd10Failed;
    }

    try self._wait_for_data_token(1000); // Wait for data token (0xFE)

    // Read 16 bytes of CID data + 2 bytes of CRC16
    var cid_data_and_crc: [18]u8 = undefined;
    _ = self.spi.read_blocking(u8, 0xFF, &cid_data_and_crc);
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
    std.log.info("Reading block {d} with size {d}", .{ block_address, block_size });

    self._cs_low();
    // CMD17 argument is the block address.
    // For SDHC/SDXC, this is directly the block number.
    // For SDSC, if CMD16 has set block length to 512, it's also the block number.
    self._send_command(CMD17, block_address, 0x29); // READ_SINGLE_BLOCK, CRC is 0x29
    const r1 = try self._read_response(1, 20);
    if (r1[0] != 0x00) {
        self._cs_high();
        return SDCardError.Cmd17Failed;
    }
    std.log.info("Got r1 {any}", .{r1});
    try self._wait_for_data_token(1000); // Wait for data token (0xFE)
    std.log.info("done waiting for data token", .{});
    // Allocate buffer for block_size bytes + 2 bytes for CRC16
    var data_and_crc_buffer: [514]u8 = undefined;

    // Perform the SPI read
    _ = self.spi.read_blocking(u8, 0xFF, &data_and_crc_buffer);
    self._cs_high();

    return data_and_crc_buffer[0..block_size];
}

fn calculate_crc16(data: []const u8) u16 {
    var crc: u16 = 0x0000; // Initial CRC value for CCITT-X.25
    const polynomial: u16 = 0x1021; // X^16 + X^12 + X^5 + 1

    for (data) |byte| {
        crc ^= @as(u16, @intCast(byte)) << 8; // XOR with the next byte shifted
        var i: u8 = 0;
        while (i < 8) : (i += 1) { // Process each bit
            if ((crc & 0x8000) != 0) { // If MSB is 1
                crc = (crc << 1) ^ polynomial; // Shift and XOR with polynomial
            } else {
                crc <<= 1; // Just shift
            }
        }
    }
    return crc;
}

/// Writes a single data block to the SD card.
///
/// Parameters:
///   block_address: The address of the block to write.
///   data: The data to write (must be 512 bytes for standard blocks).
///
/// Returns:
///   `void` on successful write, or an `SDCardError` on failure.
/// Writes a single data block to the SD card.
pub fn write_block(self: *SD_Driver, block_address: u32, data: []const u8) !void {
    if (data.len != 512) return SDCardError.InvalidDataLength;
    //if (block_address == 0) return;
    std.log.info("Writing block {d} with {d} bytes", .{ block_address, data.len });

    const arg: u32 = if (self.is_sdhc) block_address else block_address * 512;

    self._cs_low();
    defer self._cs_high(); // Ensure CS is high on function exit

    // Send WRITE_BLOCK command
    self._send_command(CMD24, arg, 0xFF); // CRC can be 0xFF in SPI mode
    const r1 = try self._read_response(1, 20);
    if (r1[0] != 0x00) {
        // If CMD24 fails, log the error code for debugging.
        std.log.err("CMD24 failed with R1 response: 0x{x}", .{r1[0]});
        return SDCardError.Cmd24Failed;
    }

    // Send one dummy byte for the gap between R1 and data token, as per spec.
    self.spi.write_blocking(u8, &[_]u8{0xFF});

    // Send data packet: Start Token + Data + Dummy CRC
    self.spi.write_blocking(u8, &[_]u8{0xFE}); // Data start token for single write
    self.spi.write_blocking(u8, data);
    self.spi.write_blocking(u8, &[_]u8{ 0xFF, 0xFF }); // Dummy CRC

    // Read data-response token.
    // The card responds with a token indicating if the data was accepted.
    // xxx00101 (0x05): Data Accepted
    // xxx01011 (0x0B): Data Rejected due to CRC error
    // xxx01101 (0x0D): Data Rejected due to a Write Error
    const resp = try self._read_response(1, 20);
    if ((resp[0] & 0x1F) != 0x05) {
        std.log.err("Data block rejected with response token: 0x{x}", .{resp[0]});
        return SDCardError.DataNotAccepted;
    }

    // Wait until card is no longer busy (MISO=0xFF).
    // The card pulls MISO low while it's busy writing to internal flash.
    // We must continuously send clocks (by sending dummy 0xFF bytes) to read this status.
    var i: u32 = 0;
    // Timeout set to ~500ms, which is a common requirement.
    const busy_timeout_clocks = 500000;
    var busy_byte: [1]u8 = .{0};
    while (i < busy_timeout_clocks) {
        self.spi.read_blocking(u8, 0xFF, &busy_byte);
        if (busy_byte[0] == 0xFF) break; // MISO is high, card is ready
        i += 1;
    }
    if (i == busy_timeout_clocks) {
        return SDCardError.CardBusyTimeout;
    }
}
/// -- now define the Disk impl that talks to your SD_Driver under the hood:
pub const Disk = struct {
    driver: *SD_Driver,
    interface: fatfs.Disk = fatfs.Disk{
        .getStatusFn = getStatus,
        .initializeFn = initializeDisk,
        .readFn = readSectors,
        .writeFn = writeSectors,
        .ioctlFn = ioctl,
    },
};

fn getStatus(_: *fatfs.Disk) fatfs.Disk.Status {
    std.log.info("Running getStatus!", .{});
    return fatfs.Disk.Status{
        .initialized = true,
        .disk_present = true,
        .write_protected = false,
    };
}

fn initializeDisk(interface: *fatfs.Disk) fatfs.Disk.Error!fatfs.Disk.Status {
    std.log.info("Running init disk!", .{});
    const self: *Disk = @fieldParentPtr("interface", interface);
    return getStatus(&self.interface);
}

fn readSectors(
    d: *fatfs.Disk,
    buff: [*]u8,
    sector: fatfs.LBA,
    count: c_uint,
) fatfs.Disk.Error!void {
    std.log.info("Running read sector!", .{});
    const self: *Disk = @fieldParentPtr("interface", d);
    const sector_size: u32 = 512;
    var off: u32 = 0;
    for (0..count) |i| {
        const addr: u32 = @intCast(sector + i);
        const data = self.driver.read_block(addr, sector_size) catch |err| {
            // Log the specific error for debugging, then return a generic one.
            std.log.err("sd read error: {s}", .{@errorName(err)});
            return fatfs.Disk.Error.DiskNotReady;
        };
        std.mem.copyForwards(u8, buff[off .. off + sector_size], data);
        off += sector_size;
    }
    return;
}

fn writeSectors(
    interface: *fatfs.Disk,
    buff: [*]const u8,
    sector: fatfs.LBA,
    count: c_uint,
) fatfs.Disk.Error!void {
    std.log.info("Running write sector! count {d}", .{count});
    const self: *Disk = @fieldParentPtr("interface", interface);
    const sector_size: u32 = 512;
    var off: u32 = 0;
    for (0..count) |i| {
        const addr: u32 = @intCast(sector + i);
        self.driver.write_block(addr, buff[off .. off + sector_size]) catch |err| {
            // Log the specific error for debugging, then return a generic one.
            std.log.err("sd write error: {s}", .{@errorName(err)});
            return fatfs.Disk.Error.DiskNotReady;
        };
        off += sector_size;
    }
    return;
}

// In src/devices/sd.zig

pub fn ioctl(interface: *fatfs.Disk, cmd: fatfs.IoCtl, buff: [*]u8) fatfs.Disk.Error!void {
    const self: *Disk = @fieldParentPtr("interface", interface);
    std.log.info("Running ioctl! with {d} sectors", .{self.driver.sector_count});

    switch (cmd) {
        .sync => {},
        .get_sector_count => {
            @as(*align(1) fatfs.LBA, @ptrCast(buff)).* = 61067264; //61067264;
        },
        else => {
            std.log.err("invalid ioctl: {}", .{cmd});
            return error.InvalidParameter;
        },
    }
}
