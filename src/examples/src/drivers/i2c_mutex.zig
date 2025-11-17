const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const i2c = rp2xxx.i2c;
const mdf = microzig.drivers;

pub const I2CMutex = struct {
    i2c_instance: i2c.I2C,
    lock: rp2xxx.mutex.Mutex,

    pub fn init(i2c_instance: i2c.I2C) I2CMutex {
        return .{
            .i2c_instance = i2c_instance,
            .lock = rp2xxx.mutex.Mutex{},
        };
    }

    pub fn apply(self: *I2CMutex, comptime config: i2c.Config) void {
        self.lock.lock();
        defer self.lock.unlock();

        self.i2c_instance.apply(config);
    }

    /// Mutex-protected wrapper for i2c.I2C.read_blocking.
    pub fn read_blocking(self: *I2CMutex, addr: i2c.Address, dst: []u8, timeout: ?mdf.time.Duration) i2c.Error!void {
        self.lock.lock();
        defer self.lock.unlock();

        return self.i2c_instance.read_blocking(addr, dst, timeout);
    }

    /// Mutex-protected wrapper for i2c.I2C.write_blocking.
    pub fn write_blocking(self: *I2CMutex, addr: i2c.Address, data: []const u8, timeout: ?mdf.time.Duration) i2c.Error!void {
        self.lock.lock();
        defer self.lock.unlock();

        return self.i2c_instance.write_blocking(addr, data, timeout);
    }

    /// Mutex-protected wrapper for i2c.I2C.write_then_read_blocking.
    pub fn write_then_read_blocking(self: *I2CMutex, addr: i2c.Address, src: []const u8, dst: []u8, timeout: ?mdf.time.Duration) i2c.Error!void {
        self.lock.lock();
        defer self.lock.unlock();

        return self.i2c_instance.write_then_read_blocking(addr, src, dst, timeout);
    }

    /// Mutex-protected wrapper for i2c.I2C.writev_then_readv_blocking.
    pub fn writev_then_readv_blocking(
        self: *I2CMutex,
        addr: i2c.Address,
        write_chunks: []const []const u8,
        read_chunks: []const []u8,
        timeout: ?mdf.time.Duration,
    ) i2c.Error!void {
        self.lock.lock();
        defer self.lock.unlock();

        return self.i2c_instance.writev_then_readv_blocking(
            addr,
            write_chunks,
            read_chunks,
            timeout,
        );
    }
};
