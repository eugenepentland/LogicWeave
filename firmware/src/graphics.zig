// graphics.zig
const std = @import("std");
const Font = @import("fonts.zig");
pub const screen_width: u16 = 248;
pub const screen_height: u16 = 128;
pub const frame_buffer_size: usize = (screen_width * screen_height) / 8;
const PbmLoader = @import("pbm_loader.zig");

// Color definitions (for monochrome)
pub const Color = enum {
    Black, // Pixel is set (ink)
    White, // Pixel is clear (paper)
};

pub const Graphics = struct {
    frame_buffer: [frame_buffer_size]u8,
    old_frame_buffer: [frame_buffer_size]u8,

    // Initializes a new Graphics instance with a white canvas.
    pub fn init() Graphics {
        var self = Graphics{
            .frame_buffer = undefined,
            .old_frame_buffer = undefined,
        };
        self.clear(Color.White);
        return self;
    }

    pub fn refreshFrameBuffer(self: *Graphics) void {
        std.mem.copyForwards(u8, self.old_frame_buffer[0..], self.frame_buffer[0..]);
    }

    pub fn getRotatedBuffer(buff: [frame_buffer_size]u8) [frame_buffer_size]u8 {
        var rotated: [frame_buffer_size]u8 = undefined;
        @memset(&rotated, 0);

        for (0..screen_height) |y_src| {
            for (0..screen_width) |x_src| {
                const bit_index_src = y_src * screen_width + x_src;
                const byte_index_src = bit_index_src / 8;
                const bit_offset_src = 7 - (x_src % 8);

                const is_black = (buff[byte_index_src] >> @as(u3, @intCast(bit_offset_src))) & 1 == 1;

                // Rotate coordinates
                const x_dst = screen_height - 1 - y_src;
                const y_dst = x_src;

                const bit_index_dst = y_dst * screen_height + x_dst;
                const byte_index_dst = bit_index_dst / 8;
                const bit_offset_dst = 7 - (x_dst % 8);

                if (is_black) {
                    rotated[byte_index_dst] |= (@as(u8, 1) << @as(u3, @intCast(bit_offset_dst)));
                }
            }
        }

        return rotated;
    }

    // Clears the entire frame buffer to the specified color.
    pub fn clear(self: *Graphics, color: Color) void {
        const fill_byte: u8 = switch (color) {
            .White => 0x00, // For e-ink, white is often all bits set (depends on display)
            .Black => 0xFF, // All bits black
        };
        @memset(&self.frame_buffer, fill_byte);
    }

    /// Draws a filled rectangle at (x, y) with given width and height in the specified color.
    pub fn drawRect(self: *Graphics, x: u16, y: u16, width: u16, height: u16, color: Color) void {
        const max_x = @min(x + width, screen_width);
        const max_y = @min(y + height, screen_height);

        var iy: u16 = y;
        while (iy < max_y) : (iy += 1) {
            var ix: u16 = x;
            while (ix < max_x) : (ix += 1) {
                self.setPixel(ix, iy, color);
            }
        }
    }

    // Sets a single pixel in the frame buffer.
    // (0,0) is top-left.
    // Note: PBM P4 format usually has MSB first. This implementation assumes MSB first.
    // A '1' bit means black, '0' means white for PBM.
    pub fn setPixel(self: *Graphics, x: u16, y: u16, color: Color) void {
        if (x >= screen_width or y >= screen_height) {
            return; // Out of bounds
        }

        const byte_index = (y * screen_width + x) / 8;
        const bit_offset = 7 - (x % 8); // MSB first: bit 7 is leftmost pixel in byte

        if (byte_index >= frame_buffer_size) return; // Should not happen if x,y are in bounds

        switch (color) {
            .Black => self.frame_buffer[byte_index] |= (@as(u8, 1) << @as(u3, @intCast(bit_offset))),
            .White => self.frame_buffer[byte_index] &= ~(@as(u8, 1) << @as(u3, @intCast(bit_offset))),
        }
    }

    fn getCharPbmData(c: u8) ?[]const u8 {
        return switch (std.ascii.toUpper(c)) {
            'A' => Font.A,
            'B' => Font.B,
            'C' => Font.C,
            'D' => Font.D,
            'E' => Font.E,
            'F' => Font.F,
            'G' => Font.G,
            'H' => Font.H,
            'I' => Font.I,
            'J' => Font.J,
            'K' => Font.K,
            'L' => Font.L,
            'M' => Font.M,
            'N' => Font.N,
            'O' => Font.O,
            'P' => Font.P,
            'Q' => Font.Q,
            'R' => Font.R,
            'S' => Font.S,
            'T' => Font.T,
            'U' => Font.U,
            'V' => Font.V,
            'W' => Font.W,
            'X' => Font.X,
            'Y' => Font.Y,
            'Z' => Font.Z,
            '.' => Font.Period,

            '0' => Font.Zero,
            '1' => Font.One,
            '2' => Font.Two,
            '3' => Font.Three,
            '4' => Font.Four,
            '5' => Font.Five,
            '6' => Font.Six,
            '7' => Font.Seven,
            '8' => Font.Eight,
            '9' => Font.Nine,

            else => null, // unsupported character
        };
    }

    pub fn drawString(g: *Graphics, text: []const u8, start_x: u16, start_y: u16) !void {
        var x_cursor = start_x;
        const space_width: u16 = 6; // Adjust this for how wide you want a space

        for (text) |c| {
            if (c == ' ') {
                x_cursor += space_width;
                continue;
            }
            const pbm_data = getCharPbmData(c) orelse continue;

            const char_img = try PbmLoader.loadFromMemory(pbm_data);
            PbmLoader.drawToGraphics(g, char_img, x_cursor, start_y);

            x_cursor += char_img.width + 2; // Add spacing between characters
        }
    }

    // Provides direct read-only access to the frame buffer.
    pub fn getFrameBuffer(self: *const Graphics) []const u8 {
        return &self.frame_buffer;
    }

    // Provides direct mutable access to the frame buffer.
    pub fn getFrameBufferMutable(self: *Graphics) []u8 {
        return &self.frame_buffer;
    }
};
