const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const peripherals = microzig.chip.peripherals;

const DEBOUCE_TIME_US: u64 = 250_000;

/// Public config type, remains the same.
pub const Config = struct {
    pin_num: u8,
    pull_state: rp2xxx.gpio.Pull,
    action_deferred: bool,
    intr_setup_name: []const u8,
    intr_clear_name: []const u8,
    field_name: []const u8,
};

/// The VTable (Virtual Method Table) holds pointers to functions
/// whose implementation depends on the comptime config.
const VTable = struct {
    setup: *const fn (self: *Interrupt) void,
    checkAndClear: *const fn (self: *Interrupt) void,
};

/// This is now a single, concrete type. All interrupt instances will have this
/// type, allowing them to be stored in an array.
pub const Interrupt = struct {
    const Self = @This();

    // Fields
    vtable: *const VTable,
    handler: *const fn () void,
    last_trigger_time: u64,
    triggered: bool,
    action_deferred: bool, // This is now a runtime value

    /// Private init function. Users should use `create()`.
    fn init(
        vtable: *const VTable,
        handler_fn: *const fn () void,
        is_deferred: bool,
    ) Self {
        return .{
            .vtable = vtable,
            .handler = handler_fn,
            .last_trigger_time = 0,
            .triggered = false,
            .action_deferred = is_deferred,
        };
    }

    /// These public methods now delegate to the vtable.
    pub fn setup(self: *Self) void {
        self.vtable.setup(self);
    }

    pub fn checkAndClear(self: *Self) void {
        self.vtable.checkAndClear(self);
    }

    /// This logic is the same for all interrupts and can stay here.
    /// It's called by the `checkAndClear` implementation from the vtable.
    fn interruptIsReady(self: *Self) void {
        const now = rp2xxx.time.get_time_since_boot().to_us();
        if ((now - self.last_trigger_time) < DEBOUCE_TIME_US) return;
        self.last_trigger_time = now;

        if (self.action_deferred) {
            self.triggered = true;
        } else {
            self.handler();
        }
    }
};

/// This function is the new factory. It takes the comptime config
/// and returns a fully initialized, concrete `Interrupt` instance.
pub fn create(comptime config: Config, handler: *const fn () void) Interrupt {
    // This helper struct generates the specific implementations for the given config.
    // It only exists at compile-time.
    const Impl = struct {
        // This is the comptime-specialized `setup` implementation.
        fn setup_impl(self: *Interrupt) void {
            _ = self; // self is not used here, but good practice to keep the signature consistent
            var pin = rp2xxx.gpio.num(config.pin_num);
            pin.set_function(.sio);
            pin.set_direction(.in);
            pin.set_pull(config.pull_state);
            pin.set_schmitt_trigger(.enabled);

            @field(peripherals.IO_BANK0, config.intr_setup_name).modify_one(config.field_name, 1);
        }

        // This is the comptime-specialized `checkAndClear` implementation.
        fn checkAndClear_impl(self: *Interrupt) void {
            var intr_reg = @field(peripherals.IO_BANK0, config.intr_clear_name);
            const intr_status = intr_reg.read();

            if (@field(intr_status, config.field_name) == 1) {
                @field(peripherals.IO_BANK0, config.intr_clear_name).modify_one(config.field_name, 1);
                // Call the shared interruptIsReady logic
                self.interruptIsReady();
            }
        }

        // Create a comptime-known, static instance of the VTable for this specific config.
        const vtable_instance = VTable{
            .setup = &setup_impl,
            .checkAndClear = &checkAndClear_impl,
        };
    };

    // Return a concrete Interrupt struct, initialized with the specialized
    // vtable and the runtime handler function.
    return Interrupt.init(&Impl.vtable_instance, handler, config.action_deferred);
}