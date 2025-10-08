const lw = @import("logicweave");
const std = @import("std");

fn echo_data(_: std.mem.Allocator, reader: *std.Io.Reader, writer: *std.Io.Writer) void {
    writer.writeAll(reader.buffered()) catch {};
}

pub fn main() !void {
    lw.custom_usb_handler = &echo_data;
    lw.run();
}
