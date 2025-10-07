const lw = @import("logicweave");
const std = @import("std");

fn echo_data(reader: std.Io.Reader) void {
    lw.usb_writer.writeAll(reader.buffer) catch {};
}

pub fn main() !void {
    lw.custom_usb_handler = &echo_data;
    lw.run();
}
