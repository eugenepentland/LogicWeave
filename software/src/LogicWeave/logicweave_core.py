
from LogicWeave import LogicWeave, GPIO, UART, I2C, SPI
from LogicWeave.logicweave import DEFAULT_GPIO_PIN
import LogicWeave.proto_gen.logicweave_core_pb2 as logicweave_core_pb2
from typing import Optional


class GpioStateError(Exception):
    """Raised when GPIO state verification fails after a write operation."""
    pass


# Voltage thresholds for state detection (adjustable at module level)
LOW_THRESHOLD = 0.8   # Below this is considered LOW
HIGH_THRESHOLD = 2.4  # Above this is considered HIGH


def _verify_pin(controller, pin: int, name: str = "pin"):
    """
    Verify a pin can be driven high and low correctly.

    Sets pin as GPIO output, writes HIGH and LOW, probing after each
    to confirm the pin isn't shorted or stuck.
    """
    if pin == DEFAULT_GPIO_PIN:
        return  # Skip unconfigured pins

    if not getattr(controller, 'debug_gpio', False):
        return

    # Create a temporary GPIO to test the pin
    temp_gpio = GPIO(controller, pin, name=f"verify_{name}")

    # Test HIGH
    temp_gpio.write(True)
    voltage = controller.probe_gpio(pin)
    if voltage < HIGH_THRESHOLD:
        raise GpioStateError(
            f"Pin {pin} ({name}) verification failed: wrote HIGH but measured {voltage:.2f}V "
            f"(expected >{HIGH_THRESHOLD}V). Pin may be shorted to ground or stuck low."
        )

    # Test LOW
    temp_gpio.write(False)
    voltage = controller.probe_gpio(pin)
    if voltage > LOW_THRESHOLD:
        raise GpioStateError(
            f"Pin {pin} ({name}) verification failed: wrote LOW but measured {voltage:.2f}V "
            f"(expected <{LOW_THRESHOLD}V). Pin may be shorted to power or stuck high."
        )


class DebugGPIO(GPIO):
    """GPIO subclass that verifies state after writes using the probe function."""

    def write(self, state: bool):
        super().write(state)

        controller = self._controller
        if not getattr(controller, 'debug_gpio', False):
            return

        voltage = controller.probe_gpio(self.pin)

        if state:  # Expected HIGH
            if voltage < HIGH_THRESHOLD:
                raise GpioStateError(
                    f"GPIO {self.pin} verification failed: wrote HIGH but measured {voltage:.2f}V "
                    f"(expected >{HIGH_THRESHOLD}V). Pin may be shorted to ground or stuck low."
                )
        else:  # Expected LOW
            if voltage > LOW_THRESHOLD:
                raise GpioStateError(
                    f"GPIO {self.pin} verification failed: wrote LOW but measured {voltage:.2f}V "
                    f"(expected <{LOW_THRESHOLD}V). Pin may be shorted to power or stuck high."
                )


class DebugUART(UART):
    """UART subclass that verifies pins before setup."""

    def _setup(self):
        controller = self._controller
        _verify_pin(controller, self.tx_pin, "tx_pin")
        _verify_pin(controller, self.rx_pin, "rx_pin")
        super()._setup()


class DebugI2C(I2C):
    """I2C subclass that verifies pins before setup."""

    def _setup(self):
        controller = self._controller
        _verify_pin(controller, self.sda_pin, "sda_pin")
        _verify_pin(controller, self.scl_pin, "scl_pin")
        super()._setup()


class DebugSPI(SPI):
    """SPI subclass that verifies pins before setup."""

    def _setup(self):
        controller = self._controller
        _verify_pin(controller, self.sclk_pin, "sclk_pin")
        _verify_pin(controller, self.mosi_pin, "mosi_pin")
        _verify_pin(controller, self.miso_pin, "miso_pin")
        super()._setup()


class LogicWeaveCore(LogicWeave):
    def __init__(self, *args, debug_gpio=True, **kwargs):
        kwargs['protobuf_module'] = logicweave_core_pb2
        super().__init__(*args, **kwargs)
        self.debug_gpio = debug_gpio

    def read_voltage(self):
        request = self.pb.ReadVoltageRequest()
        return self._send_and_parse(request, "read_voltage_response")

    def read_resistance(self):
        request = self.pb.ReadResistanceRequest()
        return self._send_and_parse(request, "read_resistance_response")
    
    def read_pd(self):
        request = self.pb.ReadPDRequest()
        return self._send_and_parse(request, "read_pd_response")
    
    def set_psu_output(self, channel, state):
        request = self.pb.SetPSUOutputRequest(channel=channel, state=state)
        return self._send_and_parse(request, "set_psu_output_response")
    
    def read_power_monitor(self):
        request = self.pb.ReadPowerMonitorRequest()
        return self._send_and_parse(request, "read_power_monitor_response")
    
    def configure_psu(self, channel, voltage, current_limit):
        request = self.pb.ConfigurePSURequest(channel=channel, voltage=voltage, current_limit=current_limit)
        return self._send_and_parse(request, "configure_psu_response")
    
    def cal_probes(self):
        request = self.pb.ZeroProbesRequest()
        return self._send_and_parse(request, "zero_probes_response")

    def read_calibration_data(self):
        request = self.pb.ReadCalibrationDataRequest()
        return self._send_and_parse(request, "read_calibration_data_response")

    def probe_gpio(self, gpio_pin: int) -> float:
        """Probe a GPIO pin and return its voltage."""
        request = self.pb.ProbeGpioRequest(gpio_pin=gpio_pin)
        response = self._send_and_parse(request, "probe_gpio_response")
        return response.voltage

    def gpio(self, pin: int, name: str = "gpio") -> DebugGPIO:
        """Create a GPIO object with optional debug verification."""
        return DebugGPIO(self, pin, name)

    def uart(self, instance_num: int, tx_pin: int = DEFAULT_GPIO_PIN,
             rx_pin: int = DEFAULT_GPIO_PIN, baud_rate: int = 115200,
             name: str = "uart") -> DebugUART:
        """Create a UART object with optional pin verification before setup."""
        return DebugUART(self, instance_num, tx_pin, rx_pin, baud_rate)

    def i2c(self, instance_num: int, sda_pin: int = DEFAULT_GPIO_PIN,
            scl_pin: int = DEFAULT_GPIO_PIN, name: str = "i2c") -> DebugI2C:
        """Create an I2C object with optional pin verification before setup."""
        return DebugI2C(self, instance_num, sda_pin, scl_pin, name)

    def spi(self, instance_num: int, sclk_pin: int, mosi_pin: int,
            miso_pin: int = DEFAULT_GPIO_PIN, baud_rate: int = 1000000,
            default_cs_pin: Optional[int] = None, name: str = "spi") -> DebugSPI:
        """Create an SPI object with optional pin verification before setup."""
        return DebugSPI(self, instance_num, sclk_pin, mosi_pin, miso_pin,
                        baud_rate, name, default_cs_pin)