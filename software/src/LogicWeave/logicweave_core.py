import time
from enum import Enum
from typing import Tuple, Optional, Any
from LogicWeave import LogicWeave, GPIO, UART, I2C, SPI
from .drivers.MCP47FEB22 import MCP47FEB22
from .drivers.AP33772S import AP33772S
from .drivers.INA700 import INA700

# --- CONSTANTS AND MAPS ---
_mux_gpio_address_map = {
    12: 4, 13: 5, 14: 6, 15: 7, 16: 3, 17: 2, 18: 1, 19: 0,
    32: 4, 33: 5, 34: 6, 35: 7, 36: 3, 37: 2, 38: 1, 39: 0,
    40: 6, 41: 7, 42: 3, 43: 2
}
TARGET_VOLTAGE = 1.65
VIN = 3.3

# --- ENUMS ---
class ResistorBank(Enum):
    """Maps descriptive names to their physical MUX address states and R_PULLUP_KNOWN."""
    # Value: (A1_state, A0_state, R_PULLUP_KNOWN)
    R_100_OHM = (False, False, 100.0)
    R_1K_OHM = (False, True, 1000.0)
    R_10K_OHM = (True, False, 10_000.0)
    R_100K_OHM = (True, True, 100_000.0)

# --- LOGIC WEAVE CORE CLASS (Main Class) ---

class LogicWeaveCore(LogicWeave):
    """
    Core class for LogicWeave system, incorporating all measurement and
    programmable power supply (PPS) functionality.
    """
    # PPS Constants for voltage-to-DAC conversion
    V_OUT_MAX = 17.0
    V_OUT_MIN = 0.7
    DAC_MAX = 4095
    DAC_MIN = 0

    def __init__(self, *args, **kwargs):
        # Call the constructor of the parent class (LogicWeave)
        super().__init__(*args, **kwargs)
        
        # --- PPS Configuration & Placeholders ---
        self.sda_pin = 0
        self.scl_pin = 1
        self.ch1_en_pin = 24
        self.ch2_en_pin = 25
        self.ch1_pm_addr = 0x46
        self.ch2_pm_addr = 0x44
        self._is_initialized = False # Tracks if PPS hardware is initialized
        
        # PPS Hardware Instances
        self.i2c_instance = None
        self.pd = None
        self.dac = None
        self.channels = {}

        # PPS MUX GPIO PIN CONFIGURATION (for self-test feature)
        self.s0_pin = 7
        self.s1_pin = 8
        self.s2_pin = 9
        self.oe0_pin = 10
        self.oe1_pin = 11
        self.oe2_pin = 23
        self.self_test_adc = 44 # ADC input for self-test
        
        # PPS MUX GPIO INSTANCE PLACEHOLDERS
        self.s0 = None
        self.s1 = None
        self.s2 = None
        self.oe0 = None
        self.oe1 = None
        self.oe2 = None
        self.adc = None

        # --- Measurement Circuit Placeholders ---
        # Voltmeter Circuit (Switch 28, ADC 46)
        self.voltmeter_switch: GPIO = None 
        self.voltmeter_adc: GPIO = None

        # Resistance Meter Circuit (Switch 27, ADC 47, MUX A0/A1 at 30/31)
        self.resistance_meter_switch: GPIO = None 
        self.resistance_adc: GPIO = None
        self.resistor_mux_a0: GPIO = None
        self.resistor_mux_a1: GPIO = None

        self._setup()

    def _get_max_fixed_pdo(self) ->  AP33772S.SRC_PDO:
        max_voltage = 0
        pdo: AP33772S.SRC_PDO = None
        for i in range(1,14):
            source_pdo = self.pd.read_source_pdo(i, False)
            if source_pdo.type == 1: continue # only check fixed pdo's
            if source_pdo.voltage_max > max_voltage:
                max_voltage = source_pdo.voltage_max
                pdo = source_pdo

        return pdo

    def _setup(self):
        """Initializes all hardware: PPS, Voltmeter, and Resistance Meter."""
        
        # --- 1. Initialize PPS Hardware (Original PPS.initialize content) ---
        print("\n--- Initializing Programmable Power Supply (PPS) Hardware ---")

        # Setup the I2C bus (self.i2c refers to the I2C instance)
        self.i2c_instance = self.i2c(instance_num=0, sda_pin=self.sda_pin, scl_pin=self.scl_pin)
        
        # Setup the PD source and request 20V input
        self.pd = AP33772S(i2c_instance=self.i2c_instance)
        pdo = self._get_max_fixed_pdo()
        pdo_req = AP33772S.PDORequest(pdo.index, pdo.current_max_code, 0x00)
        self.pd.request_pdo(pdo_req)
        print(f"PPS: PD voltage check: {self.pd.read_voltage_mv()}mV, Temp: {self.pd.read_temperature()}C")

        # Setup the DAC
        self.dac = MCP47FEB22(self.i2c_instance)
        
        # Setup channel-specific hardware (GPIOs and Power Monitors)
        self.channels = {
            1: {'en': self.gpio(self.ch1_en_pin), 'pm': INA700(self.i2c_instance, self.ch1_pm_addr)},
            2: {'en': self.gpio(self.ch2_en_pin), 'pm': INA700(self.i2c_instance, self.ch2_pm_addr)}
        }

        # MUX GPIO INITIALIZATION and SETTING HIGH
        self.s0 = self.gpio(self.s0_pin)
        self.s1 = self.gpio(self.s1_pin)
        self.s2 = self.gpio(self.s2_pin)
        self.oe0 = self.gpio(self.oe0_pin)
        self.oe1 = self.gpio(self.oe1_pin)
        self.oe2 = self.gpio(self.oe2_pin)
        self.adc = self.gpio(self.self_test_adc)

        # Set all MUX GPIOs high (e.g., disable MUXes for safety)
        self.s0.write(True)
        self.s1.write(True)
        self.s2.write(True)
        self.oe0.write(True)
        self.oe1.write(True)
        self.oe2.write(True)
        print("PPS: Initialized MUX GPIOs and set them high.")

        self._is_initialized = True
        print("--- PPS Hardware initialized successfully. ---\n")
        
        # --- 2. Initialize Measurement Hardware ---

        # Voltmeter setup
        self.voltmeter_switch = self.gpio(28)
        self.voltmeter_adc = self.gpio(46)

        # Resistance Meter setup
        self.resistance_meter_switch = self.gpio(27)
        self.resistance_adc = self.gpio(47)
        self.resistor_mux_a0 = self.gpio(30)
        self.resistor_mux_a1 = self.gpio(31)
        
        # Set all measurement circuits to a known, safe state (off)
        self.disable_all_circuits()

    # --- PPS Methods (Moved from ProgrammablePowerSupply) ---

    def _voltage_to_dac(self, voltage: float) -> int:
        """Converts a desired output voltage to a corresponding DAC value."""
        # Clamp the voltage to the safe operating range
        clamped_voltage = max(self.V_OUT_MIN, min(self.V_OUT_MAX, voltage))
        
        # Calculate DAC value using the linear formula: D = (V_MAX - V_out) * (DAC_RANGE / V_RANGE)
        voltage_range = self.V_OUT_MAX - self.V_OUT_MIN
        dac_range = self.DAC_MAX - self.DAC_MIN
        
        dac_value = (self.V_OUT_MAX - clamped_voltage) * (dac_range / voltage_range)
        
        return int(max(self.DAC_MIN, min(self.DAC_MAX, round(dac_value))))
    
    def set_channel_voltage(self, channel: int, voltage: float):
        """
        Sets the output voltage for a specific channel.
        """
        if channel not in self.channels:
            raise ValueError("Invalid channel number. Must be 1 or 2.")
        
        dac_value = self._voltage_to_dac(voltage)
        self.dac.set_voltage(channel=channel, value=dac_value)
        time.sleep(0.01)
        print(f"Measured voltage: {self.read_channel_voltage(channel):.3f}V")

        

    def set_channel_voltage_calibrated(self, channel: int, voltage: float, tolerance_v: float = 0.010, max_iterations: int = 5):
        """
        Sets the output voltage for a specific channel using a feedback loop
        to achieve high accuracy.
        """
        if channel not in self.channels:
            raise ValueError("Invalid channel number. Must be 1 or 2.")

        print(f"--- Calibrating Channel {channel} to {voltage:.3f}V (Tolerance: +/- {tolerance_v*1000:.0f}mV) ---")

        # Start with an initial guess for the DAC value
        self.enable_channel(channel, False)
        current_dac_value = self._voltage_to_dac(voltage)
        self.dac.set_voltage(channel=channel, value=current_dac_value)
        time.sleep(0.05)
        self.enable_channel(channel, True)

        KP = 200 # Proportional gain constant

        for i in range(max_iterations):
            dac_channel = channel
            self.dac.set_voltage(channel=dac_channel, value=current_dac_value)
            
            time.sleep(0.05)

            measured_voltage = self.read_channel_voltage(channel)
            error = voltage - measured_voltage

            print(f"  Iter {i+1:2d}: Target={voltage:.3f}V, Measured={measured_voltage:.3f}V, Error={error*1000:6.1f}mV, DAC={current_dac_value}")

            if abs(error) <= tolerance_v:
                print(f"--- Channel {channel} successfully set to {measured_voltage:.3f}V ---")
                return

            # Adjust DAC value
            dac_adjustment = int(error * KP)
            current_dac_value -= dac_adjustment
            
            # Clamp the DAC value
            current_dac_value = max(self.DAC_MIN, min(self.DAC_MAX, current_dac_value))

        print(f"!!! WARNING: Channel {channel} failed to calibrate to {voltage:.3f}V after {max_iterations} iterations.")
        print(f"    Final measured voltage: {self.read_channel_voltage(channel):.3f}V")


    def read_channel_voltage(self, channel: int) -> float:
        """Reads the measured output voltage from a channel's power monitor (INA700)."""
        if channel not in self.channels:
            raise ValueError("Invalid channel number. Must be 1 or 2.")
        return self.channels[channel]['pm'].read_voltage()
    
    def read_channel_current(self, channel: int) -> float:
        """Reads the measured output current from a channel's power monitor (INA700)."""
        if channel not in self.channels:
            raise ValueError("Invalid channel number. Must be 1 or 2.")
        return self.channels[channel]['pm'].read_current()

    def enable_channel(self, channel: int, state: bool = True):
        """Enables or disables a channel's output."""
        if channel not in self.channels:
            raise ValueError("Invalid channel number. Must be 1 or 2.")
        self.channels[channel]['en'].write(state)
        status = "ENABLED" if state else "DISABLED"
        print(f"PPS: Channel {channel} {status}")

    def shutdown(self):
        """Disables channels and cleans up resources for the PPS and measurement circuits."""
        print("\nLogicWeaveCore: Cleaning up resources...")
        
        # PPS Shutdown logic
        if self._is_initialized:
            try:
                # Set voltage to max (DAC 0) and disable channels for safety
                self.dac.set_voltage(channel=1, value=0)
                self.dac.set_voltage(channel=2, value=0)
                self.enable_channel(1, False)
                self.enable_channel(2, False)

                # Set MUX GPIOs low
                self.s0.write(False)
                self.s1.write(False)
                self.s2.write(False)
                self.oe0.write(False)
                self.oe1.write(False)
                self.oe2.write(False)

                print("PPS: Channels and MUX GPIOs safely disabled.")
            except Exception as e:
                print(f"PPS Warning: Could not gracefully disable channels. Error: {e}")
        
        # Measurement Circuit Shutdown logic
        self.disable_all_circuits()
        print("LogicWeaveCore: Power supply and measurement circuits offline.")

    def _set_mux_address(self, address: int):
        """Sets the S0, S1, and S2 pins to select one of the 8 Mux channels."""
        if not 0 <= address <= 7:
            raise ValueError("Mux address must be between 0 and 7.")
        
        # Extract bits (LSB to MSB: S0, S1, S2)
        s0_state = (address & 0b001) != 0
        s1_state = (address & 0b010) != 0
        s2_state = (address & 0b100) != 0

        self.s0.write(s0_state)
        self.s1.write(s1_state)
        self.s2.write(s2_state)

    def _reset_oe(self):
        """Sets all Output Enable pins high (disabling all MUX outputs)."""
        self.oe0.write(True)
        self.oe1.write(True)
        self.oe2.write(True)

    def _get_oe(self, gpio_num):
        """Returns the correct Output Enable (OE) GPIO object based on the pin number."""
        if 12 <= gpio_num <= 19:
            return self.oe0
        if 32 <= gpio_num <= 39:
            return self.oe1
        if 40 <= gpio_num <= 43:
            return self.oe2
        return None
        
    def self_test_gpio(self, gpio_num):
        """Performs a simple self-test on a specified GPIO pin via the MUX/ADC circuit."""
        pass_test = True
        self._reset_oe()
        
        # NOTE: The user's original code initialized the GPIO object here. 
        # Since the GPIO object is used temporarily, this is acceptable.
        gpio = self.gpio(gpio_num) 
        
        # Set the MUX address
        if gpio_num not in _mux_gpio_address_map:
            print(f"Error: GPIO {gpio_num} not in MUX map.")
            return False
            
        address = _mux_gpio_address_map[gpio_num]
        self._set_mux_address(address)

        oe = self._get_oe(gpio_num)
        if oe is None:
            print(f"Error: Could not determine OE pin for GPIO {gpio_num}.")
            return False

        oe.write(False) # Enable the MUX output
        # time.sleep(0.01) # Optionally wait for settling

        # Test HIGH state
        gpio.write(True)
        time.sleep(0.01)
        high_value = self.adc.read_adc()
        if high_value < 3:
            pass_test = False
            print(f"Err on gpio {gpio_num}. Set high and read {high_value}V")

        # Test LOW state
        gpio.write(False)
        time.sleep(0.01)
        low_value = self.adc.read_adc()
        if low_value > 0.3:
            pass_test = False
            print(f"Err on gpio {gpio_num}. Set low and read {low_value}V")

        self._reset_oe()
        if pass_test:
            print(f"GPIO {gpio_num} passed the self test. High Voltage: {high_value:.4f}V, Low Voltage: {low_value:.4f}V")
        return pass_test

    # --- Measurement Circuit Methods (Unchanged) ---

    def disable_all_circuits(self):
        """Disables both voltmeter and resistance meter circuits."""
        self.voltmeter_switch.write(False)
        self.resistance_meter_switch.write(False)
        
    def enable_voltmeter(self):
        """Activates the voltage divider circuit for voltage measurement."""
        self.disable_all_circuits()
        self.voltmeter_switch.write(True)
        time.sleep(0.05)

    def enable_resistance_meter(self):
        """Activates the resistor bank circuit for resistance measurement."""
        self.disable_all_circuits()
        self.resistance_meter_switch.write(True)
        time.sleep(0.05)

    def read_voltmeter(self, r1_series: float = 56000.0, r2_gnd: float = 10000.0, cal_factor: float = 1.01) -> float:
        """Reads the ADC voltage from the voltmeter circuit and scales it back to V_in."""
        self.enable_voltmeter()
        
        # Read raw Vout from the ADC (pin 46)
        vout = self.voltmeter_adc.read_adc() 
        
        # Calculate the scaling factor for the voltage divider
        scaling_factor = (r1_series + r2_gnd) / r2_gnd
        
        # Scale V_out back to the original voltage (V_in), applying calibration
        v_in = vout * scaling_factor * cal_factor
        
        self.disable_all_circuits()
        return v_in

    # --- Private Resistance Measurement Helpers ---

    def _calculate_measured_resistance(self, voltage_adc: float, r_bank: float, vin: float = VIN) -> float:
        """Calculates the unknown pulldown resistance (Rx). Rx = R_bank * V_adc / (Vin - V_adc)"""
        # Handle open circuit
        if voltage_adc <= 0.0001:
            return float('inf')

        voltage_drop_Rbank = vin - voltage_adc
        
        # Handle short circuit
        if voltage_drop_Rbank <= 0.0001:
            return 0.0

        r_x = r_bank * voltage_adc / voltage_drop_Rbank
        return r_x

    def _measure_bank_voltage(self, bank: ResistorBank) -> float:
        """Sets the MUX address and reads the ADC voltage for a specific bank."""
        try:
            # Extract A1, A0 states
            a1_state, a0_state, _ = bank.value
            
            # 1. Set the MUX address using the dedicated GPIOs (31 and 30)
            self.resistor_mux_a1.write(a1_state)
            self.resistor_mux_a0.write(a0_state)
            
            # 2. Wait for settling
            time.sleep(0.05)

            # 3. Read the ADC voltage from the resistance ADC (pin 47)
            measured_voltage = self.resistance_adc.read_adc()
            
            return measured_voltage
            
        except Exception as e:
            print(f"Error measuring bank voltage: {e}")
            return 0.0

    def read_resistance(self) -> Optional[float]:
        """
        Sweeps all banks, finds the one with V_ADC closest to TARGET_VOLTAGE (1.65V),
        and calculates the final resistance (Rx) using that bank's known value.
        """
        
        # 1. Enable the resistance circuit
        self.enable_resistance_meter()
        
        best_voltage = -1.0
        best_bank = None
        min_voltage_diff = float('inf')

        # 2. Sweep all banks to find the best one
        for bank in ResistorBank:
            measured_voltage = self._measure_bank_voltage(bank)
            
            # Calculate difference from target voltage (1.65V)
            voltage_diff = abs(measured_voltage - TARGET_VOLTAGE)
            
            if voltage_diff < min_voltage_diff:
                min_voltage_diff = voltage_diff
                best_voltage = measured_voltage
                best_bank = bank
        
        if best_bank is None:
            self.disable_all_circuits()
            return None
        
        # 3. Use the best bank's values for the final calculation
        best_r_bank = best_bank.value[2]
        calculated_resistance_rx = self._calculate_measured_resistance(
            voltage_adc=best_voltage, 
            r_bank=best_r_bank
        )
        
        # 4. Disable the circuits when done
        self.disable_all_circuits()

        return round(calculated_resistance_rx, 2)
