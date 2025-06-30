const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const spi = rp2xxx.spi;

const EPD_Driver = @This();

const UpdateMode = enum {
    Normal,
    Fast,
};

// For 2.06-inch EPD (e.g., E2206KS0E1)
const SCREEN_WIDTH_PIXELS: u16 = 248; // Vertical = wide size in C++ context
const SCREEN_HEIGHT_PIXELS: u16 = 128; // Horizontal = small size in C++ context
const TOTAL_BYTES_PER_FRAME = (SCREEN_WIDTH_PIXELS * SCREEN_HEIGHT_PIXELS) / 8; // 8 # Each bit represents 1 pixel [cite: 104

reset_pin: gpio.Pin,
busy_pin: gpio.Pin,
dc_pin: gpio.Pin,
cs_pin: gpio.Pin,
mosi_pin: gpio.Pin,
sclk_pin: gpio.Pin,
spi_instance: rp2xxx.spi.SPI, // Store the configured SPI instance
psr: [2]u8,
image_data_size: u32,

pub fn init(
    reset_p: gpio.Pin,
    busy_p: gpio.Pin,
    dc_p: gpio.Pin,
    cs_p: gpio.Pin,
    sclk_p: gpio.Pin, // Pass SCLK pin to set its function
    mosi_p: gpio.Pin, // Pass MOSI pin to set its function
    spi_inst: rp2xxx.spi.SPI, // Pass the SPI instance
) !EPD_Driver {
    // Configure Reset Pin
    reset_p.set_function(.sio);
    reset_p.set_direction(.out);
    reset_p.put(1); // Start with Reset HIGH

    // Configure Busy Pin
    busy_p.set_function(.sio);
    busy_p.set_direction(.in);
    // busy_p.pull_up(); // Optional: Depending on EPD, a pull-up might be needed if BUSY floats

    // Configure DC Pin
    dc_p.set_function(.sio);
    dc_p.set_direction(.out);
    dc_p.put(1); // Start with DC HIGH

    // Configure CS Pin
    cs_p.set_function(.sio);
    cs_p.set_direction(.out);
    cs_p.put(1); // Start with CS HIGH

    // Configure SCLK and MOSI pins for SPI function
    // This assumes these pins are correctly associated with the passed spi_inst peripheral
    sclk_p.set_function(.spi);
    mosi_p.set_function(.spi);

    // Configure the passed SPI instance
    var configured_spi = spi_inst; // Make a mutable copy if methods need *Self or return Self
    try configured_spi.apply(.{ .clock_config = rp2xxx.clock_config, .baud_rate = 8_000_000 });

    const img_data_size = @as(u32, SCREEN_WIDTH_PIXELS) * (@as(u32, SCREEN_HEIGHT_PIXELS) / 8);

    const driver = EPD_Driver{
        .reset_pin = reset_p,
        .busy_pin = busy_p,
        .dc_pin = dc_p,
        .cs_pin = cs_p,
        .sclk_pin = sclk_p,
        .mosi_pin = mosi_p,
        .spi_instance = configured_spi, // Store the configured SPI instance
        .image_data_size = img_data_size,
        .psr = [_]u8{ 0x0F, 0x0F },
    };

    return driver;
}

/// CoG initialization function (mirrors C++ EPD_Driver::COG_initial)
pub fn cog_initial(self: *@This()) void {
    // Corrected flow:
    self._reset(5, 5, 10, 5, 5); // Or your preferred EPD reset timing, e.g., matching C++ SmallP.
    self._soft_reset(); // Send 0x00, 0x0E and wait for busy.

    const psr_values = self.read_panel_settings_revised();
    self.psr = psr_values;

    // ... rest of your cog_initial ...
    self.sclk_pin.set_function(.spi); // Ensure SPI function is restored for normal operations
    self.mosi_pin.set_function(.spi);
}

/// Determines if bank 0 is active (true) or bank 1 (false)
fn determine_active_bank(self: *@This()) bool {
    const first_byte = self.read_otp_byte(0);
    return first_byte == 0xA5;
}

/// Reads the 2-byte PSR register from the OTP memory
pub fn read_panel_settings_revised(self: *@This()) [2]u8 {
    self._reset_for_otp(); // Uses (5,5,10,5,5)

    // Send OTP Read Command (0xA2)
    self.mosi_pin.set_function(.sio);
    self.mosi_pin.set_direction(.out);
    self.sclk_pin.set_function(.sio);
    self.sclk_pin.set_direction(.out);

    self.dc_pin.put(0); // Command
    self.cs_pin.put(0);
    self._bitbang_send_byte(0xA2);
    self.cs_pin.put(1);
    time.sleep_ms(10); // 10ms delay

    // Read data
    self.dc_pin.put(1); // Data mode
    _ = self._read_next_otp_data_byte(); // Dummy read
    const bank_check_byte = self._read_next_otp_data_byte();
    const is_bank0_active = (bank_check_byte == 0xA5);

    // Determine PSR addresses
    const psr0_address: u16 = if (is_bank0_active) 0x0B1B else 0x171B;

    // Skip bytes until PSR0
    var current_addr: u16 = 1; // After dummy + bank byte
    while (current_addr < psr0_address) : (current_addr += 1) {
        _ = self._read_next_otp_data_byte();
    }

    // Read PSR[0] and PSR[1]
    const psr0 = self._read_next_otp_data_byte();
    const psr1 = self._read_next_otp_data_byte();

    // Restore SPI
    self.sclk_pin.set_function(.spi);
    self.mosi_pin.set_function(.spi);

    return .{ psr0, psr1 };
}

fn _reset_for_otp(self: *@This()) void {
    self._reset(5, 5, 10, 5, 5); // Match C++ timings
}

fn _read_next_otp_data_byte(self: *@This()) u8 {
    self.cs_pin.put(0); // CS LOW to select
    const byte_val = self._bitbang_recv_byte();
    self.cs_pin.put(1); // CS HIGH to deselect/pulse
    time.sleep_us(1);
    // A small delay might be needed here if issues persist, but usually not for bit-banged SPI.
    return byte_val;
}

/// Reads a single OTP byte by bit-banging SPI at a given offset
fn read_otp_byte(self: *@This(), offset: u16) u8 {
    self._reset_for_otp();

    // 3-wire SPI mode: MOSI used for read and write
    self.mosi_pin.set_function(.sio);
    self.mosi_pin.set_direction(.out);
    self.dc_pin.put(0); // command mode
    self.cs_pin.put(0);
    self._bitbang_send_byte(0xA2); // OTP read command
    self.cs_pin.put(1);
    time.sleep_us(10); // wait per datasheet

    self.dc_pin.put(1); // data mode
    self.cs_pin.put(0);

    _ = self._bitbang_recv_byte(); // Dummy read

    var result: u8 = 0;
    for (0..offset) |_| {
        result = self._bitbang_recv_byte();
    }

    self.cs_pin.put(1);
    return result;
}

fn _bitbang_send_byte(self: *@This(), value: u8) void {
    for (0..8) |i| {
        const shift_amt: u3 = @intCast(7 - i);
        const bit = (value >> shift_amt) & 0x1;
        self.mosi_pin.put(@intCast(bit));
        self.sclk_pin.put(1);
        time.sleep_us(1);
        self.sclk_pin.put(0);
        time.sleep_us(1);
    }
}

fn _bitbang_recv_byte(self: *@This()) u8 {
    var byte: u8 = 0;

    self.mosi_pin.set_direction(.in);

    for (0..8) |_| {
        self.sclk_pin.put(1);
        time.sleep_us(1);
        const bit = self.mosi_pin.read();
        byte = (byte << 1) | bit;
        self.sclk_pin.put(0);
        time.sleep_us(1);
    }

    self.mosi_pin.set_direction(.out); // Restore to output for future writes
    return byte;
}

/// CoG shutdown function (mirrors C++ EPD_Driver::COG_powerOff)
pub fn cog_power_off(self: *@This()) void {
    self._sendCommandData8(0x02);
    self.wait_for_busy_high();
    self.dc_pin.put(0);
    self.cs_pin.put(0);
    time.sleep_ms(150);
    self.reset_pin.put(0);
}

/// Global Update function (mirrors C++ EPD_Driver::globalUpdate)
pub fn global_update(
    self: *@This(),
    old_image: []const u8,
    new_image: []const u8,
    mode: UpdateMode,
    tsset: u8, // signed 8-bit temperature
) void {
    const tsset_cmd: u8 = switch (mode) {
        .Normal => tsset,
        .Fast => tsset + 0x40,
    };
    // Set the input temperature
    self._sendCommandData(0xe5, tsset_cmd);

    // Set the active temperature
    self._sendCommandData(0xe0, 0x02);

    switch (mode) {
        .Normal => {
            self._send_index_data(0x00, self.psr[0..]);
            self._send_index_data(0x10, new_image); // DTM1
            self._send_index_data(0x13, &[_]u8{0} ** TOTAL_BYTES_PER_FRAME); // DTM2 = dummy
            self._dcdc_power_on();
            self._display_refresh();
        },
        .Fast => {
            // Panel Settings
            self._send_index_data(0x00, &[_]u8{
                self.psr[0] | 0x10,
                self.psr[1] | 0x02,
            });

            self._sendCommandData(0x50, 0x07); // VCOM/Data Interval Setting (before sending images)
            self._sendCommandData(0x50, 0x27); // VCOM/Data Interval Setting (after sending images)

            self._send_index_data(0x10, old_image); // DTM1 = old
            self._send_index_data(0x13, new_image); // DTM2 = new

            self._sendCommandData(0x50, 0x07); // VCOM/Data Interval Setting (after sending images)

            self._dcdc_power_on();
            self._display_refresh();
        },
    }
}

// ---------- PROTECTED/PRIVATE FUNCTIONS -----------

fn wait_for_busy_high(self: *@This()) void {
    while (self.busy_pin.read() == 0) {
        time.sleep_us(100); // Small delay to prevent hard spinning
    }
}

fn _send_index_data(self: *@This(), index: u8, data: []const u8) void {
    self.dc_pin.put(0);
    self.cs_pin.put(0);
    self.spi_instance.write_blocking(u8, &[_]u8{index});

    self.cs_pin.put(1);
    self.dc_pin.put(1);

    if (data.len > 0) {
        self.cs_pin.put(0);
        self.spi_instance.write_blocking(u8, data);
        self.cs_pin.put(1);
    }
}

fn _sendCommandData(self: *@This(), index: u8, data: u8) void {
    self.dc_pin.put(0);
    self.cs_pin.put(0);

    self.spi_instance.write_blocking(u8, &[_]u8{index});

    self.dc_pin.put(1);
    self.spi_instance.write_blocking(u8, &[_]u8{data});

    self.cs_pin.put(1);
}

fn _sendCommandData8(self: *@This(), index: u8) void {
    self.dc_pin.put(0);
    self.cs_pin.put(0);
    self.spi_instance.write_blocking(u8, &[_]u8{index});
    self.cs_pin.put(1);
}

fn _soft_reset(self: *@This()) void {
    self._sendCommandData(0x00, 0x0E);
    self.wait_for_busy_high();
}

fn _display_refresh(self: *@This()) void {
    self._sendCommandData8(0x12);
    self.wait_for_busy_high();
}

fn _reset(self: *@This(), ms1: u32, ms2: u32, ms3: u32, ms4: u32, ms5: u32) void {
    time.sleep_ms(ms1);
    self.reset_pin.put(1);
    time.sleep_ms(ms2);
    self.reset_pin.put(0);
    time.sleep_ms(ms3);
    self.reset_pin.put(1);
    time.sleep_ms(ms4);
    self.cs_pin.put(1);
    time.sleep_ms(ms5);
}

fn _dcdc_power_on(self: *@This()) void {
    self._sendCommandData8(0x04);
    self.wait_for_busy_high();
}
