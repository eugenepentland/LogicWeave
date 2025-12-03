import struct
import logging
import sys
import builtins
import enum
from typing import Optional, Any
from LogicWeave.exceptions import DeviceFirmwareError, DeviceResponseError, DeviceConnectionError
import LogicWeave.proto_gen.logicweave_pb2 as lw_pb2
from .transports import Transport, NativeUsbTransport, PyodideTransport, VENDOR_ID, PRODUCT_ID, INTERFACE_NUM, PACKET_SIZE

ProtobufModule = Any 

class GpioFunction(enum.IntEnum):
    """
    Defines the available functions (modes) for a GPIO pin,
    mapping directly to the integer values in the GpioFunction protobuf enum.
    """
    XIP = 0
    HSTX = 1
    SPI = 2
    UART = 3
    I2C = 4
    PWM = 5
    SIO_IN = 6
    SIO_OUT = 7
    PIO = 8
    GPCK = 9
    USB = 10
    NONE = 11

# Default GPIO value for optional, unconfigured pins.
DEFAULT_GPIO_PIN = 63

# --- Base Class for Peripherals ---
class _BasePeripheral:
    """A base class for peripheral controllers to reduce boilerplate."""
    def __init__(self, controller: 'LogicWeave'):
        self._controller = controller
        self.pb: ProtobufModule = controller.pb 

    def _build_and_execute(self, request_class, expected_response_field: str, **kwargs):
        request_payload = request_class(**kwargs)
        return self._controller._send_and_parse(request_payload, expected_response_field)

# --- Peripheral Classes ---
class UART(_BasePeripheral):
    def __init__(self, controller: 'LogicWeave', instance_num: int, 
                 tx_pin: int = DEFAULT_GPIO_PIN, rx_pin: int = DEFAULT_GPIO_PIN, 
                 baud_rate: int = 115200):
        super().__init__(controller)
        self._instance_num = instance_num
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.baud_rate = baud_rate
        self._setup()

    def _setup(self):
        self._build_and_execute(self.pb.UartSetupRequest, "uart_setup_response", 
                                instance_num=self._instance_num, tx_pin=self.tx_pin, 
                                rx_pin=self.rx_pin, baud_rate=self.baud_rate)

    def write(self, data: bytes, timeout_ms: int = 1000):
        self._build_and_execute(self.pb.UartWriteRequest, "uart_write_response", 
                                instance_num=self._instance_num, data=data, 
                                timeout_ms=timeout_ms)

    def read(self, byte_count: int, timeout_ms: int = 1000) -> bytes:
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

    def set_function(self, mode: GpioFunction):
        self._build_and_execute(self.pb.GPIOFunctionRequest, "gpio_function_response", 
                                gpio_pin=self.pin, function=mode, name=self.name)

    def set_pull(self, state: int):
        self._build_and_execute(self.pb.GpioPinPullRequest, "gpio_pin_pull_response", 
                                gpio_pin=self.pin, state=state)
        self.pull = state

    def write(self, state: bool):
        if self._controller.read_pin_function(self.pin) != self.pb.GpioFunction.sio_out:
            self.set_function(self.pb.GpioFunction.sio_out)
        self._build_and_execute(self.pb.GPIOWriteRequest, "gpio_write_response", 
                                gpio_pin=self.pin, state=state)

    def read(self) -> bool:
        if self._controller.read_pin_function(self.pin) != self.pb.GpioFunction.sio_in:
            self.set_function(self.pb.GpioFunction.sio_in)
        response = self._build_and_execute(self.pb.GPIOReadRequest, "gpio_read_response", 
                                           gpio_pin=self.pin)
        return response.state

    def setup_pwm(self, wrap, clock_div_int=0, clock_div_frac=0):
        self._build_and_execute(self.pb.PWMSetupRequest, "pwm_setup_response", 
                                gpio_pin=self.pin, wrap=wrap, 
                                clock_div_int=clock_div_int, 
                                clock_div_frac=clock_div_frac, name=self.name)

    def set_pwm_level(self, level):
        self._build_and_execute(self.pb.PWMSetLevelRequest, "pwm_set_level_response", 
                                gpio_pin=self.pin, level=level)

    def read_adc(self) -> float:
        response = self._build_and_execute(self.pb.ADCReadRequest, "adc_read_response", gpio_pin=self.pin)
        return (response.sample / self.MAX_ADC_COUNT) * self.V_REF

    def __repr__(self):
        return f"<GPIO pin={self.pin}>"

class I2C(_BasePeripheral):
    def __init__(self, controller: 'LogicWeave', instance_num: int, 
                 sda_pin: int = DEFAULT_GPIO_PIN, scl_pin: int = DEFAULT_GPIO_PIN, 
                 name: Optional[str] = "i2c"):
        super().__init__(controller)
        self._instance_num = instance_num
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.name = name
        self._setup()

    def _setup(self):
        self._build_and_execute(self.pb.I2CSetupRequest, "i2c_setup_response", 
                                instance_num=self._instance_num, sda_pin=self.sda_pin, 
                                scl_pin=self.scl_pin, name=self.name)

    def write(self, device_address: int, data: bytes):
        self._build_and_execute(self.pb.I2CWriteRequest, "i2c_write_response", 
                                instance_num=self._instance_num, 
                                device_address=device_address, data=data)

    def write_then_read(self, device_address: int, data: bytes, byte_count: int) -> bytes:
        response = self._build_and_execute(self.pb.I2CWriteThenReadRequest, "i2c_write_then_read_response", 
                                            instance_num=self._instance_num, 
                                            device_address=device_address, data=data, 
                                            byte_count=byte_count)
        return response.data

    def read(self, device_address: int, byte_count: int) -> bytes:
        response = self._build_and_execute(self.pb.I2CReadRequest, "i2c_read_response", 
                                            instance_num=self._instance_num, 
                                            device_address=device_address, 
                                            byte_count=byte_count)
        return response.data

    def __repr__(self):
        return f"<I2C instance={self._instance_num} sda={self.sda_pin} scl={self.scl_pin}>"

class SPI(_BasePeripheral):
    def __init__(self, controller: 'LogicWeave', instance_num: int, 
                 sclk_pin: int, mosi_pin: int, 
                 miso_pin: int = DEFAULT_GPIO_PIN, baud_rate: int = 1e6, 
                 name: Optional[str] = "spi", default_cs_pin: Optional[int] = None):
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
        self._build_and_execute(self.pb.SPIWriteRequest, "spi_write_response", 
                                instance_num=self._instance_num, data=data, 
                                cs_pin=self._get_cs_pin(cs_pin))

    def read(self, byte_count: int, cs_pin: Optional[int] = None, data_to_send: int = 0) -> bytes:
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

# --- Main Controller Class ---
class LogicWeave:
    def __init__(self, transport: Transport = None, protobuf_module: ProtobufModule = lw_pb2, 
                 vendor_id: int = VENDOR_ID, product_id: int = PRODUCT_ID, 
                 interface: int = INTERFACE_NUM, packet_size: int = PACKET_SIZE, 
                 timeout_ms: int = 5000, 
                 # New Logging Arguments
                 log_file: Optional[str] = None, 
                 log_console: bool = True,
                 **kwargs):
        
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.interface = interface
        self.packet_size = packet_size
        self.timeout_ms = timeout_ms
        self.pb = protobuf_module

        # --- Logging Setup ---
        # We create a logger unique to this instance to prevent collisions
        self.logger = logging.getLogger(f"LogicWeave-{id(self)}")
        self.logger.setLevel(logging.INFO)
        self.logger.handlers = [] # Reset handlers

        formatter = logging.Formatter('%(asctime)s | %(message)s', datefmt='%H:%M:%S')

        if log_console:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

        if log_file:
            file_handler = logging.FileHandler(log_file)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

        if transport is None:
            if sys.platform == 'emscripten': # 'emscripten' means we are in Pyodide
                transport = PyodideTransport
            else:
                transport = NativeUsbTransport

        # Setup Transport
        if isinstance(transport, type):
            self.transport = transport(vendor_id=self.vendor_id, 
                                     product_id=self.product_id, 
                                     interface=self.interface)
        else:
            self.transport = transport

        try:
            self.transport.open()
        except Exception as e:
            raise DeviceConnectionError(f"Connection failed: {e}")

    # --- New Logging Method ---
    def log(self, key: str, value: Any, level: int = logging.INFO):
        """
        Unified logging method.
        1. Updates Web GUI if 'update_gui' is available (injected in browser).
        2. Logs to console/file via standard logging, preserving structure.
        
        Using 'extra' allows structured loggers (like JSON formatters) to 
        access the raw 'output_key' and 'output_value' without parsing the message string.
        """
        try:
            update_gui(key, value)
        except:
            pass
        
        # B. Log to Python Logger (Console/File)
        # We pass key/value in 'extra' for better integration with logging backends
        self.logger.log(level, f"{key}: {value}", extra={'output_key': key, 'output_value': value})

    # --- Peripheral Factory Methods ---
    def uart(self, instance_num: int, tx_pin: int = DEFAULT_GPIO_PIN, rx_pin: int = DEFAULT_GPIO_PIN, baud_rate: int = 115200, name: str = "uart") -> 'UART':
        return UART(self, instance_num, tx_pin, rx_pin, baud_rate)

    def gpio(self, pin: int, name: str = "gpio") -> GPIO:
        return GPIO(self, pin, name)

    def i2c(self, instance_num: int, sda_pin: int = DEFAULT_GPIO_PIN, scl_pin: int = DEFAULT_GPIO_PIN, name: str = "i2c") -> I2C:
        return I2C(self, instance_num, sda_pin, scl_pin, name)

    def spi(self, instance_num: int, sclk_pin: int, mosi_pin: int, miso_pin: int = DEFAULT_GPIO_PIN, baud_rate: int = 1000000, default_cs_pin: Optional[int] = None, name: str = "spi") -> SPI:
        return SPI(self, instance_num, sclk_pin, mosi_pin, miso_pin, baud_rate, name, default_cs_pin)

    def _execute_transaction(self, specific_message_payload):
        app_message = self.pb.RequestMessage()
        
        field_name = None
        for field in app_message.DESCRIPTOR.fields:
            if field.containing_oneof and field.message_type == specific_message_payload.DESCRIPTOR:
                field_name = field.name
                break
        
        if not field_name:
            raise ValueError(f"Could not find a field in RequestMessage for: {type(specific_message_payload).__name__}.")
        
        getattr(app_message, field_name).CopyFrom(specific_message_payload)
        
        request_bytes = app_message.SerializeToString()
        length = len(request_bytes)
        
        MAX_PAYLOAD_SIZE = self.packet_size - 1
        if length > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Message too large: {length} bytes.")

        length_prefix = struct.pack("B", length) 
        padding_needed = self.packet_size - (1 + length)
        padding = b'\x00' * padding_needed
        packet_to_send = length_prefix + request_bytes + padding

        try:
            self.transport.write(packet_to_send)
            full_response_bytes = self.transport.read(self.packet_size, 5000)
        except Exception as e:
            raise DeviceConnectionError(f"USB Transfer Error: {e}") from e
        
        if len(full_response_bytes) == 0:
            return self.pb.ResponseMessage() 

        response_length = full_response_bytes[0]
        response_bytes = full_response_bytes[1 : 1 + response_length]
        
        try:
            parsed_response = self.pb.ResponseMessage()
            parsed_response.ParseFromString(response_bytes)
            return parsed_response
        except Exception as e:
            raise DeviceFirmwareError(f"Client-side parse error: {e}")

    def _send_and_parse(self, request_payload, expected_response_field: str):
        response_app_msg = self._execute_transaction(request_payload)
        response_field = response_app_msg.WhichOneof("kind")
        if response_field == "error_response":
            raise DeviceFirmwareError(f"{response_app_msg.error_response.message}")
        if response_field != expected_response_field:
            raise DeviceResponseError(expected=expected_response_field, received=response_field)
        return getattr(response_app_msg, response_field)

    def close(self):
        if self.transport:
            self.transport.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # --- High-Level API Methods ---
    def read_firmware_info(self) -> 'ProtobufModule.FirmwareInfoResponse':
        request = self.pb.FirmwareInfoRequest()
        return self._send_and_parse(request, "firmware_info_response")

    def write_bootloader_request(self):
        request = self.pb.UsbBootloaderRequest(val=1)
        self._send_and_parse(request, "usb_bootloader_response")

    def read_pin_function(self, gpio_pin):
        request = self.pb.GPIOReadFunctionRequest(gpio_pin=gpio_pin)
        return self._send_and_parse(request, "gpio_read_function_response")