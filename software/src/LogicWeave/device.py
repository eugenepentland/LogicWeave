import serial
import time
import struct
import LogicWeave.proto_gen.all_pb2 as all_pb2
from LogicWeave.exceptions import DeviceFirmwareError, DeviceResponseError, DeviceConnectionError

import serial.tools.list_ports
from typing import Optional


# --- Base Class for Peripherals ---
class _BasePeripheral:
    """A base class for peripheral controllers to reduce boilerplate."""
    def __init__(self, controller: 'LogicWeave'):
        self._controller = controller

    def _build_and_execute(self, request_class, expected_response_field: str, **kwargs):
        """A helper to build the request object, send it, and parse the response."""
        request_payload = request_class(**kwargs)
        return self._controller._send_and_parse(request_payload, expected_response_field)


# --- Peripheral Classes ---
class UART(_BasePeripheral):
    """Represents a configured UART peripheral instance."""
    def __init__(self, controller: 'LogicWeave', instance_num: int, tx_pin: int, rx_pin: int, baud_rate: int):
        super().__init__(controller)
        self._instance_num = instance_num
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.baud_rate = baud_rate
        self._setup()

    def _setup(self):
        self._build_and_execute(all_pb2.UartSetupRequest, "uart_setup_response", 
                                instance_num=self._instance_num, tx_pin=self.tx_pin, 
                                rx_pin=self.rx_pin, baud_rate=self.baud_rate)

    def write(self, data: bytes, timeout_ms: int = 1000):
        self._build_and_execute(all_pb2.UartWriteRequest, "uart_write_response", 
                                instance_num=self._instance_num, data=data, 
                                timeout_ms=timeout_ms)

    def read(self, byte_count: int, timeout_ms: int = 1000) -> bytes:
        response = self._build_and_execute(all_pb2.UartReadRequest, "uart_read_response", 
                                          instance_num=self._instance_num, 
                                          byte_count=byte_count, timeout_ms=timeout_ms)
        return response.data

    def __repr__(self):
        return f"<UART instance={self._instance_num} tx={self.tx_pin} rx={self.rx_pin} baud={self.baud_rate}>"


class GPIO(_BasePeripheral):
    MAX_ADC_COUNT = 4095
    V_REF = 3.3

    def __init__(self, controller: 'LogicWeave', pin: int):
        super().__init__(controller)
        self.pin = pin
        self.pull = None

    def set_function(self, mode: all_pb2.GpioFunction):
        self._build_and_execute(all_pb2.GPIOFunctionRequest, "gpio_function_response", 
                                gpio_pin=self.pin, function=mode)

    def set_pull(self, state: all_pb2.PinPullState):
        self._build_and_execute(all_pb2.GpioPinPullRequest, "gpio_pin_pull_response", 
                                gpio_pin=self.pin, state=state)
        self.pull = state

    def write(self, state: bool):
        if self._controller.read_pin_function(self.pin) != all_pb2.GpioFunction.sio_out:
            self.set_function(all_pb2.GpioFunction.sio_out)
        self._build_and_execute(all_pb2.GPIOWriteRequest, "gpio_write_response", 
                                gpio_pin=self.pin, state=state)

    def read(self) -> bool:
        if self._controller.read_pin_function(self.pin) != all_pb2.GpioFunction.sio_in:
            self.set_function(all_pb2.GpioFunction.sio_in)
        response = self._build_and_execute(all_pb2.GPIOReadRequest, "gpio_read_response", 
                                          gpio_pin=self.pin)
        return response.state

    def setup_pwm(self, wrap, clock_div_int=0, clock_div_frac=0):
        self._build_and_execute(all_pb2.PWMSetupRequest, "pwm_setup_response", 
                                gpio_pin=self.pin, wrap=wrap, 
                                clock_div_int=clock_div_int, 
                                clock_div_frac=clock_div_frac)

    def set_pwm_level(self, level):
        self._build_and_execute(all_pb2.PWMSetLevelRequest, "pwm_set_level_response", 
                                gpio_pin=self.pin, level=level)

    def read_adc(self) -> float:
        response = self._build_and_execute(all_pb2.ADCReadRequest, "adc_read_response", 
                                          gpio_pin=self.pin)
        return (response.sample / self.MAX_ADC_COUNT) * self.V_REF

    def __repr__(self):
        return f"<GPIO pin={self.pin}>"


class I2C(_BasePeripheral):
    def __init__(self, controller: 'LogicWeave', instance_num: int, sda_pin: int, scl_pin: int):
        super().__init__(controller)
        self._instance_num = instance_num
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self._setup()

    def _setup(self):
        self._build_and_execute(all_pb2.I2CSetupRequest, "i2c_setup_response", 
                                instance_num=self._instance_num, sda_pin=self.sda_pin, 
                                scl_pin=self.scl_pin)

    def write(self, device_address: int, data: bytes):
        self._build_and_execute(all_pb2.I2CWriteRequest, "i2c_write_response", 
                                instance_num=self._instance_num, 
                                device_address=device_address, data=data)

    def write_then_read(self, device_address: int, data: bytes, byte_count: int) -> bytes:
        response = self._build_and_execute(all_pb2.I2CWriteThenReadRequest, "i2c_write_then_read_response", 
                                          instance_num=self._instance_num, 
                                          device_address=device_address, data=data, 
                                          byte_count=byte_count)
        return response.data

    def read(self, device_address: int, byte_count: int) -> bytes:
        response = self._build_and_execute(all_pb2.I2CReadRequest, "i2c_read_response", 
                                          instance_num=self._instance_num, 
                                          device_address=device_address, 
                                          byte_count=byte_count)
        return response.data

    def __repr__(self):
        return f"<I2C instance={self._instance_num} sda={self.sda_pin} scl={self.scl_pin}>"


class SPI(_BasePeripheral):
    def __init__(self, controller: 'LogicWeave', instance_num: int, sclk_pin: int, mosi_pin: int, miso_pin: int, baud_rate: int, default_cs_pin: Optional[int] = None):
        super().__init__(controller)
        self._instance_num = instance_num
        self.sclk_pin = sclk_pin
        self.mosi_pin = mosi_pin
        self.miso_pin = miso_pin
        self.baud_rate = baud_rate
        self._default_cs_pin = default_cs_pin
        self._setup()

    def _setup(self):
        self._build_and_execute(all_pb2.SPISetupRequest, "spi_setup_response", 
                                instance_num=self._instance_num, sclk_pin=self.sclk_pin, 
                                mosi_pin=self.mosi_pin, miso_pin=self.miso_pin, 
                                baud_rate=self.baud_rate)

    def _get_cs_pin(self, cs_pin_override: Optional[int]) -> int:
        active_cs_pin = cs_pin_override if cs_pin_override is not None else self._default_cs_pin
        if active_cs_pin is None: 
            raise ValueError("A Chip Select (CS) pin must be provided.")
        return active_cs_pin

    def write(self, data: bytes, cs_pin: Optional[int] = None):
        self._build_and_execute(all_pb2.SPIWriteRequest, "spi_write_response", 
                                instance_num=self._instance_num, data=data, 
                                cs_pin=self._get_cs_pin(cs_pin))

    def read(self, byte_count: int, cs_pin: Optional[int] = None, data_to_send: int = 0) -> bytes:
        response = self._build_and_execute(all_pb2.SPIReadRequest, "spi_read_response", 
                                          instance_num=self._instance_num, 
                                          data=data_to_send, 
                                          cs_pin=self._get_cs_pin(cs_pin), 
                                          byte_count=byte_count)
        return response.data

    def __repr__(self):
        parts = [f"<SPI instance={self._instance_num}", f"sclk={self.sclk_pin}", f"mosi={self.mosi_pin}", f"miso={self.miso_pin}"]
        if self._default_cs_pin is not None: 
            parts.append(f"default_cs={self._default_cs_pin}")
        return " ".join(parts) + ">"


def _get_device_port():
    """Finds the serial port for the LogicWeave device by VID/PID."""
    vid, pid = 0x1E8B, 0x0001
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == vid and port.pid == pid:
            return port.device
    return None


# --- Main Controller Class ---
class LogicWeave:
    """A high-level wrapper for communicating with the LogicWeave device over serial."""
    def __init__(self, port: Optional[str] = None, baudrate=115200, timeout=1, write_delay=0, **kwargs):
        self.write_delay = write_delay
        
        if not port:
            port = _get_device_port()
            if not port:
                raise DeviceConnectionError("Could not auto-detect device. Please specify a serial port.")
        
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, **kwargs)
        except serial.SerialException as e:
            raise DeviceConnectionError(f"Failed to connect to {port}: {e}") from e

        # --- THIS SET IS NO LONGER NEEDED ---
        # self._EMPTY_RESPONSE_FIELDS = { ... }

    # --- Peripheral Factory Methods ---
    def uart(self, instance_num: int, tx_pin: int, rx_pin: int, baud_rate: int = 115200) -> 'UART':
        return UART(self, instance_num, tx_pin, rx_pin, baud_rate)

    def gpio(self, pin: int) -> GPIO:
        return GPIO(self, pin)

    def i2c(self, instance_num: int, sda_pin: int, scl_pin: int) -> I2C:
        return I2C(self, instance_num, sda_pin, scl_pin)

    def spi(self, instance_num: int, sclk_pin: int, mosi_pin: int, miso_pin: int, baud_rate: int = 1000000, default_cs_pin: Optional[int] = None) -> SPI:
        return SPI(self, instance_num, sclk_pin, mosi_pin, miso_pin, baud_rate, default_cs_pin)

    # --- Core Communication Logic ---
    def _execute_transaction(self, specific_message_payload):
        app_message = all_pb2.AppMessage()
        
        field_name = None
        for field in app_message.DESCRIPTOR.fields:
            if field.containing_oneof and field.message_type == specific_message_payload.DESCRIPTOR:
                field_name = field.name
                break
        
        if not field_name:
            raise ValueError(f"Could not find a field in AppMessage for message type: {type(specific_message_payload).__name__}")
        
        getattr(app_message, field_name).CopyFrom(specific_message_payload)
        
        request_bytes = app_message.SerializeToString()
        length = len(request_bytes)
        if length > 256:
            raise ValueError(f"Message too large for 1-byte prefix: {length} bytes.")

        length_prefix = struct.pack(">B", length)
        
        if not self.ser or not self.ser.is_open:
            raise DeviceConnectionError("Serial port is not open.")

        # Write request
        self.ser.reset_input_buffer()
        self.ser.write(length_prefix + request_bytes)
        if self.write_delay > 0:
            time.sleep(self.write_delay)

        # Read response length
        response_length_byte = self.ser.read(1)
        if not response_length_byte:
            # Timeout occurred
            return all_pb2.AppMessage() 

        response_length = response_length_byte[0]
        
        # Read response body
        response_bytes = self.ser.read(response_length)
        if len(response_bytes) != response_length:
            raise DeviceResponseError(f"Incomplete response. Expected {response_length}, got {len(response_bytes)}.")

        # Parse response
        try:
            parsed_response = all_pb2.AppMessage()
            parsed_response.ParseFromString(response_bytes)
            return parsed_response
        except Exception as e:
            raise DeviceFirmwareError(f"Client-side parse error: {e}. Raw data: {response_bytes.hex()}")

    def _send_and_parse(self, request_payload, expected_response_field: str):
        """Sends a request and parses the expected response, simplifying error handling."""
        response_app_msg = self._execute_transaction(request_payload)
        response_field = response_app_msg.WhichOneof("kind")

        if response_field == "error_response":
            raise DeviceFirmwareError(f"Device error: {response_app_msg.error_response.message}")

        # --- Handle expected empty responses dynamically ---
        if response_field is None:
            try:
                # Check if the expected response message type is defined as empty in the .proto file
                field_descriptor = all_pb2.AppMessage.DESCRIPTOR.fields_by_name[expected_response_field]
                is_truly_empty = len(field_descriptor.message_type.fields) == 0
                
                if is_truly_empty:
                    # This is an expected empty response (e.g. for a simple write command),
                    # so a lack of response body is considered success.
                    return all_pb2.Empty()
            except KeyError:
                # This is a programming error, but we let the check below handle it as a mismatch.
                pass 

        if response_field != expected_response_field:
            raise DeviceResponseError(expected=expected_response_field, received=response_field)

        return getattr(response_app_msg, response_field)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # --- High-Level API Methods ---
    def read_firmware_info(self) -> all_pb2.FirmwareInfoResponse:
        request = all_pb2.FirmwareInfoRequest(info=1)
        return self._send_and_parse(request, "firmware_info_response")

    def write_bootloader_request(self):
        request = all_pb2.UsbBootloaderRequest(val=1)
        self._send_and_parse(request, "usb_bootloader_response")

    def read_pin_function(self, gpio_pin):
        request = all_pb2.GPIOReadFunctionRequest(gpio_pin=gpio_pin)
        return self._send_and_parse(request, "gpio_read_function_response")