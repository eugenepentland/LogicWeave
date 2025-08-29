const hardware = @import("hardware.zig");
const Graphics = @import("graphics.zig");

pub const ActionFn = fn () void;

pub const MenuItem = struct {
    name: []const u8,
    children: []const *MenuItem,
    parent: ?*MenuItem,
    action: ?*ActionFn,
};

pub const MenuSystem = struct {
    root_menu: *MenuItem,
    current_menu: *MenuItem,
    selected_index: u8,
};

var sd_load: MenuItem = .{
    .name = "Load from SD",
    .children = &[_]*MenuItem{&something_else},
    .parent = &home_menu,
    .action = undefined,
};

var something_else: MenuItem = .{
    .name = "Something else",
    .children = &[_]*MenuItem{},
    .parent = &home_menu,
    .action = undefined,
};

var home_menu: MenuItem = .{
    .name = "Home",
    .children = &[_]*MenuItem{ &sd_load, &something_else },
    .parent = undefined,
    .action = undefined,
};

pub var Menu: MenuSystem = .{
    .root_menu = &home_menu,
    .current_menu = &home_menu,
    .selected_index = 0,
};

pub fn render_menu() !void {
    hardware.g.clear(.White);
    for (Menu.current_menu.children, 0..) |menu, i| {
        const y_pos: u16 = @as(u16, @intCast(i)) * 30 + 5;
        try hardware.g.drawString(menu.name, 5, y_pos);
    }
    try hardware.update_screen();
}
