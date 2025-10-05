# LogicWeave Firmware

**Universal RP2040/RP2350 firmware controllable via Python or Protocol Buffers over USB**

LogicWeave Firmware is a modular, high-performance embedded firmware that runs on any RP2040 or RP2350 microcontroller. It provides a unified communication layer that allows full control of GPIOs, peripherals, and board-level features through a Python API or via Protocol Buffers (protobuf) messages exchanged over USB. This makes it ideal for applications such as automated test systems, hardware validation, and device control setups.

LogicWeave is written in Zig and built on top of the MicroZig hardware abstraction layer, ensuring portability, performance, and clean architecture. The firmware exposes an extensible command framework that can be easily expanded with new peripherals, sensors, or custom message types.

## Features
- Works with **any RP2040 or RP2350 board**
- **USB communication** supporting both serial and bulk transfer
- **Python library (`logicweave`)** for high-level control
- **Protobuf-based USB protocol** for low-level or multi-language integration
- Supports **GPIO, SPI, I²C, UART, PWM, ADC**, and other peripherals
- Easily extendable for custom hardware or new commands

## Getting Started

### Flashing the Firmware
Build and flash LogicWeave firmware using Zig:
```bash
cd src/examples
zig build
````

Then copy the generated `.uf2` or `.bin` file to your board’s USB drive.

### Installing the Python Library

```bash
pip install logicweave
```

### Connecting from Python

Once flashed, connect your board via USB. It will enumerate as a serial or USB bulk device.

```python
from LogicWeave import LogicWeave

with LogicWeave() as lw:
    print("Connected to LogicWeave firmware!")
    info = lw.read_firmware_info()
    print(f"Firmware Version: {info.version}")
```

## Examples

### GPIO Control

```python
from LogicWeave import LogicWeave, GPIOMode
import time

with LogicWeave() as lw:
    led = lw.gpio(25)
    led.set_mode(GPIOMode.OUTPUT)
    for _ in range(5):
        led.write(True)
        time.sleep(0.5)
        led.write(False)
        time.sleep(0.5)
```

### I²C Communication

```python
with LogicWeave() as lw:
    i2c = lw.i2c(instance_num=0, sda_pin=4, scl_pin=5)
    i2c.write(0x27, b'\x01\x02')
    data = i2c.read(0x27, 2)
    print(f"Received: {data.hex()}")
```

### SPI Communication

```python
with LogicWeave() as lw:
    spi = lw.spi(instance_num=0, sclk_pin=6, mosi_pin=7, miso_pin=8, baud_rate=1_000_000)
    spi.write(b'\xAA\x55')
    rx = spi.read(2)
    print(f"RX: {rx.hex()}")
```

## Protobuf Interface

For low-level or cross-language applications, LogicWeave supports direct communication via Protocol Buffers. All commands and responses are defined in `.proto` files and exchanged over USB endpoints. Each message includes an `id` field for deterministic command tracking.

```protobuf
message Command {
  uint32 id = 1;
  oneof payload {
    GpioWrite gpio_write = 2;
    I2cTransaction i2c_transaction = 3;
    SpiTransfer spi_transfer = 4;
  }
}
```

## Extending LogicWeave

LogicWeave is written in Zig and designed to be easily extended:

1. Add a new `.proto` message definition.
2. Regenerate Zig and Python protobuf bindings.
3. Implement the corresponding command handler in `firmware/src/handlers/`.
4. Rebuild and flash the firmware.

This modular design makes LogicWeave suitable for a wide range of embedded control and test automation scenarios.

## License

Licensed under the **MIT License**. See `LICENSE.md` for details.