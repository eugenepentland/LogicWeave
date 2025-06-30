const std = @import("std");
const Graphics = @import("graphics.zig");

pub const LoadedPbm = struct {
    width: u16,
    height: u16,
    data: []const u8, // raw pixel bytes, 1bpp, padded per row
};

pub fn loadFromMemory(embedded_data: []const u8) !LoadedPbm {
    if (!std.mem.startsWith(u8, embedded_data, "P4\n")) {
        return error.InvalidPbmFormat;
    }

    // Find start of pixel data by counting newlines
    var newline_count: usize = 0;
    var pixel_data_offset: usize = 0;
    while (pixel_data_offset < embedded_data.len) : (pixel_data_offset += 1) {
        if (embedded_data[pixel_data_offset] == '\n') {
            newline_count += 1;
            if (newline_count == 2) {
                pixel_data_offset += 1;
                break;
            }
        }
    }
    if (newline_count < 2) return error.InvalidPbmFormat;

    // Parse width/height
    var tokenizer = std.mem.tokenizeAny(u8, embedded_data, " \n");
    _ = tokenizer.next(); // P4
    const width = try std.fmt.parseUnsigned(u16, tokenizer.next() orelse return error.InvalidPbmFormat, 10);
    const height = try std.fmt.parseUnsigned(u16, tokenizer.next() orelse return error.InvalidPbmFormat, 10);

    const bytes_per_row = (width + 7) / 8;
    const pixel_data_len = bytes_per_row * height;

    if (pixel_data_offset + pixel_data_len > embedded_data.len) {
        return error.EndOfFileUnexpected;
    }

    const slice = embedded_data[pixel_data_offset .. pixel_data_offset + pixel_data_len];

    return LoadedPbm{
        .width = width,
        .height = height,
        .data = slice, // zero-copy: points to embedded memory
    };
}

pub fn drawToGraphics(g: *Graphics.Graphics, pbm: LoadedPbm, x_offset: u16, y_offset: u16) void {
    const bytes_per_row = (pbm.width + 7) / 8;

    for (0..pbm.height) |y_src| {
        for (0..pbm.width) |x_src| {
            const byte_index = y_src * bytes_per_row + (x_src / 8);
            const bit_offset = 7 - (x_src % 8);

            const byte = pbm.data[byte_index];
            const is_black = ((byte >> @as(u3, @intCast(bit_offset))) & 1) != 0;

            const x = x_offset + @as(u16, @intCast(x_src));
            const y = y_offset + @as(u16, @intCast(y_src));

            if (x < Graphics.screen_width and y < Graphics.screen_height) {
                g.setPixel(x, y, if (is_black) .Black else .White);
            }
        }
    }
}
