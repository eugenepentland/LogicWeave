class MCP47FEB22:
    """
    A controller class for the MCP47FEB22A2-E_ST Dual 12-Bit DAC.

    This class provides a simple interface to control the DAC's output
    voltage using the LogicWeave I2C driver.
    """
    # From the datasheet (Table 6-1), the A2 variant has the I2C address 1100010b.
    DEVICE_ADDRESS = 0x62

    # From the datasheet (Table 4-1), Volatile DAC register addresses.
    _VOLATILE_DAC_REGISTERS = {
        2: 0x00,  # DAC Channel 0
        1: 0x01,  # DAC Channel 1
    }

    def __init__(self, i2c_bus):
        """
        Initializes the MCP47FEB22 controller.

        Args:
            i2c_bus: An initialized I2C object from the LogicWeave driver.
        """
        self.i2c = i2c_bus

    def set_voltage(self, channel, value):
        """
        Sets the output voltage for a specific DAC channel.

        The output voltage is determined by the formula:
        V_out = (V_ref * value) / 4096

        Args:
            channel (int): The DAC channel to control (0 or 1).
            value (int): The 12-bit digital value (0-4095) to set.
        """
        if channel not in self._VOLATILE_DAC_REGISTERS:
            print(self._VOLATILE_DAC_REGISTERS, channel)
            raise ValueError("Channel must be 0 or 1.")

        if not 0 <= value <= 4095:
            raise ValueError("Value must be a 12-bit integer (0-4095).")

        # Get the memory address for the specified DAC channel.
        mem_address = self._VOLATILE_DAC_REGISTERS[channel]

        # From the datasheet (Figure 7-1), the command byte for a "Write Command" is
        # constructed as: [AD4, AD3, AD2, AD1, AD0, C1, C0, x]
        # Where C1,C0 = '00' for a write command and x is a reserved bit.
        # This simplifies to (mem_address << 3).
        command_byte = mem_address << 3

        # For the 12-bit DAC, the data is sent in two bytes.
        # The upper 4 bits of the first byte are ignored (set to 0).
        # Data Byte 1: [0, 0, 0, 0, D11, D10, D9, D8]
        # Data Byte 2: [D7, D6, D5, D4, D3, D2, D1, D0]
        data_byte_1 = (value >> 8) & 0x0F
        data_byte_2 = value & 0xFF

        # The payload is the command byte followed by the two data bytes.
        payload = bytes([command_byte, data_byte_1, data_byte_2])

        self.i2c.write(device_address=self.DEVICE_ADDRESS, data=payload)