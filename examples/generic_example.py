from LogicWeave import LogicWeave

with LogicWeave() as lw:
    # Print the firmware version
    print(lw.read_firmware_info())

    # GPIO
    led = lw.gpio(3)
    led.write(True)
    value = led.read()
    print(value)

    # SPI
    spi = lw.spi(instance_num=0, sclk_pin=2, mosi_pin=3, miso_pin=4, baud_rate=100_000, default_cs_pin=7)
    # Writes 8 bits at a time, using the default CS pin
    spi.write(bytes([0xFF, 0x0F]))
    # Write using the same SPI bus but a different CS pin
    spi.write(bytes([0xFF, 0x0F]), cs_pin=10)
    # Reading SPI
    spi.read(byte_count=4)

    # I2C
    i2c = lw.i2c(instance_num=0, sda_pin=0, scl_pin=1)
    i2c.write(device_address=0xFF, data=bytes([0xFF]))
    value = i2c.read(device_address=0xFF, byte_count=1)

    # UART
    uart = lw.uart(instance_num=0, tx_pin=0, rx_pin=1)
    uart.write(bytes([0xFF, 0xFF]))
    value = uart.read(byte_count=4)

