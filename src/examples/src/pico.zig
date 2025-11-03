const logicweave = @import("logicweave");
const messages = @import("lw_core");
const lw = logicweave.init(messages);

pub fn main() !void {
    lw.run();
}
