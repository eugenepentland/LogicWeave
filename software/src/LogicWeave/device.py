import serial
import time
import struct
# Removed: import LogicWeave.proto_gen.logicweave_pb2 as all_pb2
from LogicWeave.exceptions import DeviceFirmwareError, DeviceResponseError, DeviceConnectionError

import serial.tools.list_ports
from typing import Optional, Any
# Add a type hint for the protobuf module to improve clarity
ProtobufModule = Any 


# --- Base Class for Peripherals ---
class _BasePeripheral:
    """A base class for peripheral controllers to reduce boilerplate."""
    def __init__(self, controller: 'LogicWeave'):
        self._controller = controller
        # Store a direct reference to the protobuf module
        self.pb: ProtobufModule = controller.pb 

    def _build_and_execute(self, request_class, expected_response_field: str, **kwargs):
        """A helper to build the request object, send it, and parse the response."""
        # request_class is now passed as the protobuf message *type* (e.g., self.pb.UartSetupRequest)
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
        # Now uses self.pb
        self._build_and_execute(self.pb.UartSetupRequest, "uart_setup_response", 
                                instance_num=self._instance_num, tx_pin=self.tx_pin, 
                                rx_pin=self.rx_pin, baud_rate=self.baud_rate)

    def write(self, data: bytes, timeout_ms: int = 1000):
        # Now uses self.pb
        self._build_and_execute(self.pb.UartWriteRequest, "uart_write_response", 
                                instance_num=self._instance_num, data=data, 
                                timeout_ms=timeout_ms)

    def read(self, byte_count: int, timeout_ms: int = 1000) -> bytes:
        # Now uses self.pb
        response = self._build_and_execute(self.pb.UartReadRequest, "uart_read_response", 
                                            instance_num=self._instance_num, 
                                            byte_count=byte_count, timeout_ms=timeout_ms)
        return response.data

    def __repr__(self):
        return f"<UART instance={self._instance_num} tx={self.tx_pin} rx={self.rx_pin} baud={self.baud_rate}>"


class GPIO(_BasePeripheral):
    MAX_ADC_COUNT = 4095
    V_REF = 3.3

    def __init__(self, controller: 'LogicWeave', pin: int, name: Optional[str] = "gpio"):
        super().__init__(controller)
        self.pin = pin
        self.pull = None
        self.name = name

    def set_function(self, mode: int): # Type hint changed to int as the enum is not imported globally
        # Now uses self.pb
        self._build_and_execute(self.pb.GPIOFunctionRequest, "gpio_function_response", 
                                gpio_pin=self.pin, function=mode, name=self.name)

    def set_pull(self, state: int): # Type hint changed to int as the enum is not imported globally
        # Now uses self.pb
        self._build_and_execute(self.pb.GpioPinPullRequest, "gpio_pin_pull_response", 
                                gpio_pin=self.pin, state=state)
        self.pull = state

    def write(self, state: bool):
        # Now uses self.pb
        if self._controller.read_pin_function(self.pin) != self.pb.GpioFunction.sio_out:
            self.set_function(self.pb.GpioFunction.sio_out)
        self._build_and_execute(self.pb.GPIOWriteRequest, "gpio_write_response", 
                                gpio_pin=self.pin, state=state)

    def read(self) -> bool:
        # Now uses self.pb
        if self._controller.read_pin_function(self.pin) != self.pb.GpioFunction.sio_in:
            self.set_function(self.pb.GpioFunction.sio_in)
        response = self._build_and_execute(self.pb.GPIOReadRequest, "gpio_read_response", 
                                            gpio_pin=self.pin)
        return response.state

    def setup_pwm(self, wrap, clock_div_int=0, clock_div_frac=0):
        # Now uses self.pb
        self._build_and_execute(self.pb.PWMSetupRequest, "pwm_setup_response", 
                                gpio_pin=self.pin, wrap=wrap, 
                                clock_div_int=clock_div_int, 
                                clock_div_frac=clock_div_frac, name=self.name)

    def set_pwm_level(self, level):
        # Now uses self.pb
        self._build_and_execute(self.pb.PWMSetLevelRequest, "pwm_set_level_response", 
                                gpio_pin=self.pin, level=level)

    def read_adc(self) -> float:
        # Now uses self.pb
        response = self._build_and_execute(self.pb.ADCReadRequest, "adc_read_response", 
                                            gpio_pin=self.pin)
        return (response.sample / self.MAX_ADC_COUNT) * self.V_REF

    def __repr__(self):
        return f"<GPIO pin={self.pin}>"


class I2C(_BasePeripheral):
    def __init__(self, controller: 'LogicWeave', instance_num: int, sda_pin: int, scl_pin: int, name: Optional[str] = "i2c"):
        super().__init__(controller)
        self._instance_num = instance_num
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.name = name
        self._setup()

    def _setup(self):
        # Now uses self.pb
        self._build_and_execute(self.pb.I2CSetupRequest, "i2c_setup_response", 
                                instance_num=self._instance_num, sda_pin=self.sda_pin, 
                                scl_pin=self.scl_pin, name=self.name)

    def write(self, device_address: int, data: bytes):
        # Now uses self.pb
        self._build_and_execute(self.pb.I2CWriteRequest, "i2c_write_response", 
                                instance_num=self._instance_num, 
                                device_address=device_address, data=data)

    def write_then_read(self, device_address: int, data: bytes, byte_count: int) -> bytes:
        # Now uses self.pb
        response = self._build_and_execute(self.pb.I2CWriteThenReadRequest, "i2c_write_then_read_response", 
                                            instance_num=self._instance_num, 
                                            device_address=device_address, data=data, 
                                            byte_count=byte_count)
        return response.data

    def read(self, device_address: int, byte_count: int) -> bytes:
        # Now uses self.pb
        response = self._build_and_execute(self.pb.I2CReadRequest, "i2c_read_response", 
                                            instance_num=self._instance_num, 
                                            device_address=device_address, 
                                            byte_count=byte_count)
        return response.data

    def __repr__(self):
        return f"<I2C instance={self._instance_num} sda={self.sda_pin} scl={self.scl_pin}>"


class SPI(_BasePeripheral):
    def __init__(self, controller: 'LogicWeave', instance_num: int, sclk_pin: int, mosi_pin: int, miso_pin: int, baud_rate: int, name: Optional[str] = "spi", default_cs_pin: Optional[int] = None):
        super().__init__(controller)
        self._instance_num = instance_num
        self.sclk_pin = sclk_pin
        self.mosi_pin = mosi_pin
        self.miso_pin = miso_pin
        self.baud_rate = baud_rate
        self._default_cs_pin = default_cs_pin
        self.name = name
        self._setup()

    def _setup(self):
        # Now uses self.pb
        self._build_and_execute(self.pb.SPISetupRequest, "spi_setup_response", 
                                instance_num=self._instance_num, sclk_pin=self.sclk_pin, 
                                mosi_pin=self.mosi_pin, miso_pin=self.miso_pin, 
                                baud_rate=self.baud_rate, name=self.name)

    def _get_cs_pin(self, cs_pin_override: Optional[int]) -> int:
        active_cs_pin = cs_pin_override if cs_pin_override is not None else self._default_cs_pin
        if active_cs_pin is None: 
            raise ValueError("A Chip Select (CS) pin must be provided.")
        return active_cs_pin

    def write(self, data: bytes, cs_pin: Optional[int] = None):
        # Now uses self.pb
        self._build_and_execute(self.pb.SPIWriteRequest, "spi_write_response", 
                                instance_num=self._instance_num, data=data, 
                                cs_pin=self._get_cs_pin(cs_pin))

    def read(self, byte_count: int, cs_pin: Optional[int] = None, data_to_send: int = 0) -> bytes:
        # Now uses self.pb
        response = self._build_and_execute(self.pb.SPIReadRequest, "spi_read_response", 
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
        if port.vid == vid and port.pid == pid and port.interface == "LogicWeave Driver":
            return port.device
    return None


# --- Main Controller Class ---
class LogicWeave:
    """A high-level wrapper for communicating with the LogicWeave device over serial."""
    def __init__(self, protobuf_module: ProtobufModule, port: Optional[str] = None, baudrate=115200, timeout=1, write_delay=0, **kwargs):
        # 🌟 Store the user-provided protobuf module
        self.pb = protobuf_module
        self.write_delay = write_delay
        
        if not port:
            port = _get_device_port()
            if not port:
                raise DeviceConnectionError("Could not auto-detect device. Please specify a serial port.")
        
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, **kwargs)
        except serial.SerialException as e:
            raise DeviceConnectionError(f"Failed to connect to {port}: {e}") from e

    # --- Peripheral Factory Methods ---
    def uart(self, instance_num: int, tx_pin: int, rx_pin: int, baud_rate: int = 115200, name: str = "uart") -> 'UART':
        return UART(self, instance_num, tx_pin, rx_pin, baud_rate)

    def gpio(self, pin: int, name: str = "gpio") -> GPIO:
        return GPIO(self, pin, name)

    def i2c(self, instance_num: int, sda_pin: int, scl_pin: int, name: str = "i2c") -> I2C:
        return I2C(self, instance_num, sda_pin, scl_pin, name)

    def spi(self, instance_num: int, sclk_pin: int, mosi_pin: int, miso_pin: int, baud_rate: int = 1000000, default_cs_pin: Optional[int] = None, name: str = "spi") -> SPI:
        return SPI(self, instance_num, sclk_pin, mosi_pin, miso_pin, baud_rate, default_cs_pin, name)

    # --- Core Communication Logic ---
    def _execute_transaction(self, specific_message_payload):
        # Now uses self.pb
        app_message = self.pb.RequestMessage()
        
        field_name = None
        for field in app_message.DESCRIPTOR.fields:
            if field.containing_oneof and field.message_type == specific_message_payload.DESCRIPTOR:
                field_name = field.name
                break
        
        if not field_name:
            # This check is crucial for handling custom messages the user adds.
            raise ValueError(f"Could not find a field in AppMessage for message type: {type(specific_message_payload).__name__}. "
                             f"Did you forget to add this message type to the 'kind' oneof in your .proto file?")
        
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
            # Timeout occurred - Now returns an empty AppMessage from the injected module
            return self.pb.ResponseMessage() 

        response_length = response_length_byte[0]
        
        # Read response body
        response_bytes = self.ser.read(response_length)
        if len(response_bytes) != response_length:
            raise DeviceResponseError(f"Incomplete response. Expected {response_length}, got {len(response_bytes)}.")

        # Parse response
        try:
            # Now uses self.pb
            parsed_response = self.pb.ResponseMessage()
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
                # Now uses self.pb
                field_descriptor = self.pb.ResponseMessage.DESCRIPTOR.fields_by_name[expected_response_field]
                is_truly_empty = len(field_descriptor.message_type.fields) == 0
                
                if is_truly_empty:
                    # Now uses self.pb
                    return self.pb.Empty()
            except KeyError:
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
    def read_firmware_info(self) -> 'ProtobufModule.FirmwareInfoResponse': # Type hint updated
        # Now uses self.pb
        request = self.pb.FirmwareInfoRequest(info=1)
        return self._send_and_parse(request, "firmware_info_response")

    def write_bootloader_request(self):
        # Now uses self.pb
        request = self.pb.UsbBootloaderRequest(val=1)
        self._send_and_parse(request, "usb_bootloader_response")

    def read_pin_function(self, gpio_pin):
        # Now uses self.pb
        request = self.pb.GPIOReadFunctionRequest(gpio_pin=gpio_pin)
        return self._send_and_parse(request, "gpio_read_function_response")