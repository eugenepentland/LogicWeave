# web.py
import struct
import asyncio
import logging  # <--- Added logging import
from typing import Optional, Any
import sys

# Pyodide specific import (only available in browser)
try:
    import js
except ImportError:
    js = None

import LogicWeave.proto_gen.logicweave_pb2 as lw_pb2
from .transports import Transport, VENDOR_ID, PRODUCT_ID, INTERFACE_NUM, PACKET_SIZE
from .exceptions import DeviceConnectionError, DeviceFirmwareError, DeviceResponseError
# Import the base LogicWeave class to inherit constants/structure if needed, 
# though we largely redefine behaviors for async.
from .logicweave import LogicWeave as SyncLogicWeave

ProtobufModule = Any

# ==========================================
# 1. Async Web Transport
# ==========================================
class WebUSBTransport(Transport):
    """
    Adapts the browser's navigator.usb via the global 'window.usbBridge' 
    object defined in your HTML.
    """
    def __init__(self, js_interface_name="usbBridge", **kwargs):
        if not js:
            raise RuntimeError("WebUSBTransport can only run in a Pyodide/Browser environment.")
        self.js = js
        self.bridge = getattr(js, js_interface_name)

    async def ensure_open(self):
        if not self.bridge.isConnected():
            raise DeviceConnectionError("WebUSB device is not connected in the browser.")

    # In web, 'open' is handled by the UI permission prompt, not Python
    def open(self):
        pass

    async def write(self, data: bytes):
        # Convert Python bytes to JS Uint8Array
        js_array = self.js.Uint8Array.new(data)
        await self.bridge.writePacket(js_array)

    async def read(self, length: int, timeout_ms: int) -> bytes:
        # Returns a JS Uint8Array, convert back to bytes
        response_js = await self.bridge.readPacket(length, timeout_ms)
        return bytes(response_js.to_py())

    async def close(self):
        # Browser handles connection lifecycle mostly
        pass


# ==========================================
# 2. Async Base Peripheral
# ==========================================
class _AsyncBasePeripheral:
    """Async version of _BasePeripheral."""
    def __init__(self, controller: 'WebLogicWeave'):
        self._controller = controller
        self.pb: ProtobufModule = controller.pb

    async def _build_and_execute(self, request_class, expected_response_field: str, **kwargs):
        request_payload = request_class(**kwargs)
        return await self._controller._send_and_parse(request_payload, expected_response_field)


# ==========================================
# 3. Async Peripheral Implementations
# ==========================================

class AsyncUART(_AsyncBasePeripheral):
    def __init__(self, controller: 'WebLogicWeave', instance_num: int, tx_pin: int, rx_pin: int, baud_rate: int):
        super().__init__(controller)
        self._instance_num = instance_num
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.baud_rate = baud_rate
        # Async setup task
        asyncio.create_task(self._setup())

    async def _setup(self):
        await self._build_and_execute(self.pb.UartSetupRequest, "uart_setup_response", 
                                      instance_num=self._instance_num, tx_pin=self.tx_pin, 
                                      rx_pin=self.rx_pin, baud_rate=self.baud_rate)

    async def write(self, data: bytes, timeout_ms: int = 1000):
        await self._build_and_execute(self.pb.UartWriteRequest, "uart_write_response", 
                                      instance_num=self._instance_num, data=data, 
                                      timeout_ms=timeout_ms)

    async def read(self, byte_count: int, timeout_ms: int = 1000) -> bytes:
        response = await self._build_and_execute(self.pb.UartReadRequest, "uart_read_response", 
                                                 instance_num=self._instance_num, 
                                                 byte_count=byte_count, timeout_ms=timeout_ms)
        return response.data

    def __repr__(self):
        return f"<AsyncUART instance={self._instance_num} tx={self.tx_pin} rx={self.rx_pin}>"


class AsyncGPIO(_AsyncBasePeripheral):
    MAX_ADC_COUNT = 4095
    V_REF = 3.3

    def __init__(self, controller: 'WebLogicWeave', pin: int, name: Optional[str] = "gpio"):
        super().__init__(controller)
        self.pin = pin
        self.pull = None
        self.name = name

    async def set_function(self, mode: int):
        await self._build_and_execute(self.pb.GPIOFunctionRequest, "gpio_function_response", 
                                      gpio_pin=self.pin, function=mode, name=self.name)

    async def set_pull(self, state: int):
        await self._build_and_execute(self.pb.GpioPinPullRequest, "gpio_pin_pull_response", 
                                      gpio_pin=self.pin, state=state)
        self.pull = state

    async def write(self, state: bool):
        # Async check for current function
        current_func = await self._controller.read_pin_function(self.pin)
        if current_func != self.pb.GpioFunction.sio_out:
             await self.set_function(self.pb.GpioFunction.sio_out)
        
        await self._build_and_execute(self.pb.GPIOWriteRequest, "gpio_write_response", 
                                      gpio_pin=self.pin, state=state)

    async def read(self) -> bool:
        current_func = await self._controller.read_pin_function(self.pin)
        if current_func != self.pb.GpioFunction.sio_in:
             await self.set_function(self.pb.GpioFunction.sio_in)
             
        response = await self._build_and_execute(self.pb.GPIOReadRequest, "gpio_read_response", 
                                                 gpio_pin=self.pin)
        return response.state

    async def setup_pwm(self, wrap, clock_div_int=0, clock_div_frac=0):
        await self._build_and_execute(self.pb.PWMSetupRequest, "pwm_setup_response", 
                                      gpio_pin=self.pin, wrap=wrap, 
                                      clock_div_int=clock_div_int, 
                                      clock_div_frac=clock_div_frac, name=self.name)

    async def set_pwm_level(self, level):
        await self._build_and_execute(self.pb.PWMSetLevelRequest, "pwm_set_level_response", 
                                      gpio_pin=self.pin, level=level)

    async def read_adc(self) -> float:
        response = await self._build_and_execute(self.pb.ADCReadRequest, "adc_read_response", 
                                                 gpio_pin=self.pin)
        return (response.sample / self.MAX_ADC_COUNT) * self.V_REF

    def __repr__(self):
        return f"<AsyncGPIO pin={self.pin}>"


class AsyncI2C(_AsyncBasePeripheral):
    def __init__(self, controller: 'WebLogicWeave', instance_num: int, sda_pin: int, scl_pin: int, name: Optional[str] = "i2c"):
        super().__init__(controller)
        self._instance_num = instance_num
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.name = name
        # Async setup task
        asyncio.create_task(self._setup())

    async def _setup(self):
        await self._build_and_execute(self.pb.I2CSetupRequest, "i2c_setup_response", 
                                      instance_num=self._instance_num, sda_pin=self.sda_pin, 
                                      scl_pin=self.scl_pin, name=self.name)

    async def write(self, device_address: int, data: bytes):
        await self._build_and_execute(self.pb.I2CWriteRequest, "i2c_write_response", 
                                      instance_num=self._instance_num, 
                                      device_address=device_address, data=data)

    async def write_then_read(self, device_address: int, data: bytes, byte_count: int) -> bytes:
        response = await self._build_and_execute(self.pb.I2CWriteThenReadRequest, "i2c_write_then_read_response", 
                                                 instance_num=self._instance_num, 
                                                 device_address=device_address, data=data, 
                                                 byte_count=byte_count)
        return response.data

    async def read(self, device_address: int, byte_count: int) -> bytes:
        response = await self._build_and_execute(self.pb.I2CReadRequest, "i2c_read_response", 
                                                 instance_num=self._instance_num, 
                                                 device_address=device_address, 
                                                 byte_count=byte_count)
        return response.data

    def __repr__(self):
        return f"<AsyncI2C instance={self._instance_num} sda={self.sda_pin} scl={self.scl_pin}>"


class AsyncSPI(_AsyncBasePeripheral):
    def __init__(self, controller: 'WebLogicWeave', instance_num: int, sclk_pin: int, mosi_pin: int, miso_pin: int, baud_rate: int, name: Optional[str] = "spi", default_cs_pin: Optional[int] = None):
        super().__init__(controller)
        self._instance_num = instance_num
        self.sclk_pin = sclk_pin
        self.mosi_pin = mosi_pin
        self.miso_pin = miso_pin
        self.baud_rate = baud_rate
        self._default_cs_pin = default_cs_pin
        self.name = name
        # Async setup task
        asyncio.create_task(self._setup())

    async def _setup(self):
        await self._build_and_execute(self.pb.SPISetupRequest, "spi_setup_response", 
                                      instance_num=self._instance_num, sclk_pin=self.sclk_pin, 
                                      mosi_pin=self.mosi_pin, miso_pin=self.miso_pin, 
                                      baud_rate=self.baud_rate, name=self.name)

    def _get_cs_pin(self, cs_pin_override: Optional[int]) -> int:
        # This helper remains sync as it's just logic
        active_cs_pin = cs_pin_override if cs_pin_override is not None else self._default_cs_pin
        if active_cs_pin is None: 
            raise ValueError("A Chip Select (CS) pin must be provided.")
        return active_cs_pin

    async def write(self, data: bytes, cs_pin: Optional[int] = None):
        await self._build_and_execute(self.pb.SPIWriteRequest, "spi_write_response", 
                                      instance_num=self._instance_num, data=data, 
                                      cs_pin=self._get_cs_pin(cs_pin))

    async def read(self, byte_count: int, cs_pin: Optional[int] = None, data_to_send: int = 0) -> bytes:
        response = await self._build_and_execute(self.pb.SPIReadRequest, "spi_read_response", 
                                                 instance_num=self._instance_num, 
                                                 data=data_to_send, 
                                                 cs_pin=self._get_cs_pin(cs_pin), 
                                                 byte_count=byte_count)
        return response.data

    def __repr__(self):
        return f"<AsyncSPI instance={self._instance_num}>"


# ==========================================
# 4. Main Async Controller Class
# ==========================================
class WebLogicWeave(SyncLogicWeave):
    """
    An Async/Await version of the LogicWeave controller designed for the Web/Pyodide.
    It inherits from SyncLogicWeave to keep constant/init signature compatibility,
    but overrides all operational methods to be async.
    """
    def __init__(self, transport=None, protobuf_module=lw_pb2, 
                 vendor_id=VENDOR_ID, product_id=PRODUCT_ID, 
                 interface=INTERFACE_NUM, packet_size=PACKET_SIZE, 
                 timeout_ms=5000, 
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

        # --- Logging Setup (Ported from LogicWeave) ---
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
            # Note: In Pyodide, this writes to the in-memory Emscripten filesystem
            file_handler = logging.FileHandler(log_file)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

        # Force WebUSBTransport if not provided
        if transport is None:
            self.transport = WebUSBTransport(vendor_id=self.vendor_id, 
                                             product_id=self.product_id)
        else:
            self.transport = transport

    # --- New Logging Method (Ported from LogicWeave) ---
    def log(self, key: str, value: Any, level: int = logging.INFO):
        """
        Unified logging method.
        1. Updates Web GUI if 'update_gui' is available.
        2. Logs to console/file via standard logging, preserving structure.
        """
        try:
            # Check for global update_gui function (common in Pyodide/GUI implementations)
            update_gui(key, value)
        except:
            pass
        
        # Log to Python Logger (Console/File)
        self.logger.log(level, f"{key}: {value}", extra={'output_key': key, 'output_value': value})

    # --- Async Context Manager ---
    async def __aenter__(self):
        await self.transport.ensure_open()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()

    async def close(self):
        await self.transport.close()

    # --- Overridden Factory Methods (Return Async Wrappers) ---
    def uart(self, instance_num: int, tx_pin: int, rx_pin: int, baud_rate: int = 115200, name: str = "uart") -> AsyncUART:
        return AsyncUART(self, instance_num, tx_pin, rx_pin, baud_rate)

    def gpio(self, pin: int, name: str = "gpio") -> AsyncGPIO:
        return AsyncGPIO(self, pin, name)

    def i2c(self, instance_num: int, sda_pin: int, scl_pin: int, name: str = "i2c") -> AsyncI2C:
        return AsyncI2C(self, instance_num, sda_pin, scl_pin, name)

    def spi(self, instance_num: int, sclk_pin: int, mosi_pin: int, miso_pin: int, baud_rate: int = 1000000, default_cs_pin: Optional[int] = None, name: str = "spi") -> AsyncSPI:
        return AsyncSPI(self, instance_num, sclk_pin, mosi_pin, miso_pin, baud_rate, name, default_cs_pin)

    # --- Core Async Execution Logic ---
    async def _execute_transaction(self, specific_message_payload):
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
            # AWAIT HERE: Transport write
            await self.transport.write(packet_to_send)
            # AWAIT HERE: Transport read
            full_response_bytes = await self.transport.read(self.packet_size, 5000)
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

    async def _send_and_parse(self, request_payload, expected_response_field: str):
        # AWAIT HERE
        response_app_msg = await self._execute_transaction(request_payload)
        
        response_field = response_app_msg.WhichOneof("kind")
        if response_field == "error_response":
             raise DeviceFirmwareError(f"{response_app_msg.error_response.message}")
        if response_field != expected_response_field:
             raise DeviceResponseError(expected=expected_response_field, received=response_field)
        return getattr(response_app_msg, response_field)

    # --- High-Level Async API Methods ---
    async def read_firmware_info(self) -> 'ProtobufModule.FirmwareInfoResponse':
        request = self.pb.FirmwareInfoRequest()
        return await self._send_and_parse(request, "firmware_info_response")

    async def write_bootloader_request(self):
        request = self.pb.UsbBootloaderRequest(val=1)
        await self._send_and_parse(request, "usb_bootloader_response")

    async def read_pin_function(self, gpio_pin):
        request = self.pb.GPIOReadFunctionRequest(gpio_pin=gpio_pin)
        return await self._send_and_parse(request, "gpio_read_function_response")
    
import LogicWeave.proto_gen.logicweave_core_pb2 as logicweave_core_pb2

# Inherit from the Async Web Controller you defined previously
class WebLogicWeaveCore(WebLogicWeave):
    """
    Async implementation of LogicWeaveCore for Browser/Pyodide.
    Inherits from WebLogicWeave to get the Async WebUSB transport
    and creates async wrappers for the Core specific functionality.
    """
    def __init__(self, *args, **kwargs):
        # Inject the extended protobuf definitions (core_pb2) 
        # so self.pb has the voltage/resistance/etc message definitions
        kwargs['protobuf_module'] = logicweave_core_pb2
        super().__init__(*args, **kwargs)

    async def read_voltage(self):
        request = self.pb.ReadVoltageRequest()
        return await self._send_and_parse(request, "read_voltage_response")

    async def read_resistance(self):
        request = self.pb.ReadResistanceRequest()
        return await self._send_and_parse(request, "read_resistance_response")
    
    async def read_pd(self):
        request = self.pb.ReadPDRequest()
        return await self._send_and_parse(request, "read_pd_response")
    
    async def set_psu_output(self, channel, state):
        request = self.pb.SetPSUOutputRequest(channel=channel, state=state)
        return await self._send_and_parse(request, "set_psu_output_response")
    
    async def read_power_monitor(self):
        request = self.pb.ReadPowerMonitorRequest()
        return await self._send_and_parse(request, "read_power_monitor_response")
    
    async def configure_psu(self, channel, voltage, current_limit):
        request = self.pb.ConfigurePSURequest(
            channel=channel, 
            voltage=voltage, 
            current_limit=current_limit
        )
        return await self._send_and_parse(request, "configure_psu_response")
    
    async def cal_probes(self):
        request = self.pb.ZeroProbesRequest()
        return await self._send_and_parse(request, "zero_probes_response")

    async def read_calibration_data(self):
        request = self.pb.ReadCalibrationDataRequest()
        return await self._send_and_parse(request, "read_calibration_data_response")