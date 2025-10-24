class INA700:
    """
    A class to interface with the INA700 Digital Power Monitor via I2C.
    """
    # Register Addresses from INA700 Datasheet
    REG_CONFIG = 0x00
    REG_VBUS = 0x05
    REG_DIETEMP = 0x06
    REG_CURRENT = 0x07
    REG_POWER = 0x08
    REG_ENERGY = 0x09
    REG_CHARGE = 0x0A
    REG_MANUFACTURER_ID = 0x3E

    # LSB conversion factors from Datasheet tables
    VOLTAGE_LSB = 0.003125      # 3.125 mV/LSB
    CURRENT_LSB = 0.000480      # 480 µA/LSB
    TEMPERATURE_LSB = 0.125     # 125 m°C/LSB
    POWER_LSB = 0.000096        # 96 µW/LSB
    ENERGY_LSB = 0.001536       # 1.536 mJ/LSB
    CHARGE_LSB = 0.000030       # 30 µC/LSB

    def __init__(self, i2c_bus, device_address=0x44):
        """
        Initializes the INA700 driver.

        Args:
            i2c_bus: An initialized I2C object from the LogicWeave driver.
            device_address (int): The I2C address of the INA700 device.
                                  Default is 0x44 (A0 pin to GND).
        """
        self.i2c = i2c_bus
        self.address = device_address

    def _read_register(self, register_address, byte_count):
        """
        Reads a value from a specified register with a given byte count.

        Args:
            register_address (int): The address of the register to read from.
            byte_count (int): The number of bytes to read.

        Returns:
            int: The value read from the register.
        """
        raw_data = self.i2c.write_then_read(
            device_address=self.address,
            data=bytes([register_address]),
            byte_count=byte_count
        )
        return int.from_bytes(raw_data, 'big')

    def _write_register_16bit(self, register_address, value):
        """
        Writes a 16-bit value to a specified register.

        Args:
            register_address (int): The address of the register to write to.
            value (int): The 16-bit value to write.
        """
        data_to_write = bytes([register_address]) + value.to_bytes(2, 'big')
        self.i2c.write(device_address=self.address, data=data_to_write)

    def reset(self):
        """
        Triggers a software reset of the INA700, equivalent to a power-on reset.
        """
        # Setting bit 15 of the CONFIG register to 1 triggers a reset.
        self._write_register_16bit(self.REG_CONFIG, 0x8000)
    
    def reset_accumulators(self):
        """
        Resets the ENERGY and CHARGE accumulation registers to 0.
        """
        # Setting bit 14 of the CONFIG register to 1 resets accumulators.
        self._write_register_16bit(self.REG_CONFIG, 0x4000)

    def read_manufacturer_id(self):
        """
        Reads the manufacturer ID from the device.
        Expected value is 0x5449 (ASCII for "TI").

        Returns:
            int: The manufacturer ID.
        """
        return self._read_register(self.REG_MANUFACTURER_ID, 2)

    def read_voltage(self):
        """
        Reads the bus voltage.

        Returns:
            float: The bus voltage in Volts (V).
        """
        raw_voltage = self._read_register(self.REG_VBUS, 2)
        # The VBUS register is always positive.
        voltage = raw_voltage * self.VOLTAGE_LSB
        return voltage

    def read_current(self):
        """
        Reads the current from the shunt resistor.

        Returns:
            float: The current in Amperes (A).
        """
        raw_current = self._read_register(self.REG_CURRENT, 2)
        # Convert from 16-bit two's complement
        if raw_current & (1 << 15):
            # If the MSB is 1, it's a negative number
            raw_current -= (1 << 16)
            
        current = raw_current * self.CURRENT_LSB
        return abs(current)

    def read_temperature(self):
        """
        Reads the internal die temperature.

        Returns:
            float: The temperature in degrees Celsius (°C).
        """
        raw_temp = self._read_register(self.REG_DIETEMP, 2)
        # Temp is a 12-bit two's complement value in the upper bits (15-4)
        temp_val = raw_temp >> 4
        if temp_val & (1 << 11):
             temp_val -= (1 << 12)
        
        temperature = temp_val * self.TEMPERATURE_LSB
        return temperature

    def read_power(self):
        """
        Reads the calculated power.

        Returns:
            float: The power in Watts (W).
        """
        # Power is an unsigned 24-bit value
        raw_power = self._read_register(self.REG_POWER, 3)
        power = raw_power * self.POWER_LSB
        return power

    def read_energy(self):
        """
        Reads the accumulated energy.

        Returns:
            float: The energy in Joules (J).
        """
        # Energy is an unsigned 40-bit value
        raw_energy = self._read_register(self.REG_ENERGY, 5)
        energy = raw_energy * self.ENERGY_LSB
        return energy
        
    def read_charge(self):
        """
        Reads the accumulated charge.

        Returns:
            float: The charge in Coulombs (C).
        """
        # Charge is a signed 40-bit two's complement value
        raw_charge = self._read_register(self.REG_CHARGE, 5)
        if raw_charge & (1 << 39):
            raw_charge -= (1 << 40)
        charge = raw_charge * self.CHARGE_LSB
        return charge
