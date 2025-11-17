const logicweave = @import("logicweave");
const messages = @import("lw_core");
const lw = logicweave.init(messages);
const microzig = @import("microzig");
const rp2xxx = microzig.hal;

pub fn main() !void {
    lw.setup();
    const usb_dev = rp2xxx.usb.Usb(.{});

    while (true) {
        usb_dev.task(false) catch unreachable;
        lw.handleUsbRx();
        lw.handleUsbTx();
    }
}
