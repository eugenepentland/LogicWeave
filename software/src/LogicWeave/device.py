import usb.core
import usb.util
import time
import struct
# Removed: import LogicWeave.proto_gen.logicweave_pb2 as all_pb2
from LogicWeave.exceptions import DeviceFirmwareError, DeviceResponseError, DeviceConnectionError
import LogicWeave.proto_gen.logicweave_pb2 as lw_pb2
from typing import Optional, Any
# Add a type hint for the protobuf module to improve clarity
ProtobufModule = Any 

VENDOR_ID = 0x2E8A
PRODUCT_ID = 0x000a
INTERFACE_NUM = 0
PACKET_SIZE = 64 # Must match the fixed size in your protocol

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


def _find_usb_device(vendor_id: int, product_id: int) -> Optional[usb.core.Device]:
    """Finds the LogicWeave device by VID/PID."""
    # Find our device
    dev = usb.core.find(idVendor=vendor_id, idProduct=product_id)
    return dev


# --- Main Controller Class ---
class LogicWeave:
    """A high-level wrapper for communicating with the LogicWeave device over a custom USB vendor interface."""
    def __init__(self, protobuf_module: ProtobufModule = lw_pb2, 
                 vendor_id: int = VENDOR_ID, product_id: int = PRODUCT_ID, 
                 interface: int = INTERFACE_NUM, packet_size: int = PACKET_SIZE, 
                 timeout_ms: int = 5000, **kwargs):
        
        self.pb = protobuf_module
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.interface = interface
        self.packet_size = packet_size
        self.timeout_ms = timeout_ms
        self.dev: Optional[usb.core.Device] = None
        self.ep_out: Optional[usb.core.Endpoint] = None
        self.ep_in: Optional[usb.core.Endpoint] = None
        self.kernel_driver_detached = False

        self._setup_usb_connection()

    def _setup_usb_connection(self):
        """Initializes the USB connection using pyusb."""
        # 1. Find the device
        self.dev = _find_usb_device(self.vendor_id, self.product_id)

        if self.dev is None:
            raise DeviceConnectionError(f"Device not found. Is it plugged in and enumerated?")

        try:
            # 2. Detach kernel driver if active
            if self.dev.is_kernel_driver_active(self.interface):
                self.dev.detach_kernel_driver(self.interface)
                self.kernel_driver_detached = True

            # 3. Set configuration and get interface
            self.dev.set_configuration()
            cfg = self.dev.get_active_configuration()
            intf = cfg[(self.interface, 0)]

            # 4. Find IN and OUT endpoints
            self.ep_out = usb.util.find_descriptor(
                intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
            )
            self.ep_in = usb.util.find_descriptor(
                intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
            )

            if self.ep_out is None or self.ep_in is None:
                self.dev = None
                raise DeviceConnectionError("Could not find IN and OUT endpoints.")

        except usb.core.USBError as e:
            self.dev = None # Ensure cleanup runs cleanly if an error occurs here
            raise DeviceConnectionError(f"USB Setup Error: {e}") from e

    # --- Peripheral Factory Methods ---
    def uart(self, instance_num: int, tx_pin: int, rx_pin: int, baud_rate: int = 115200, name: str = "uart") -> 'UART':
        return UART(self, instance_num, tx_pin, rx_pin, baud_rate)

    def gpio(self, pin: int, name: str = "gpio") -> GPIO:
        return GPIO(self, pin, name)

    def i2c(self, instance_num: int, sda_pin: int, scl_pin: int, name: str = "i2c") -> I2C:
        return I2C(self, instance_num, sda_pin, scl_pin, name)

    def spi(self, instance_num: int, sclk_pin: int, mosi_pin: int, miso_pin: int, baud_rate: int = 1000000, default_cs_pin: Optional[int] = None, name: str = "spi") -> SPI:
        return SPI(self, instance_num, sclk_pin, mosi_pin, miso_pin, baud_rate, name, default_cs_pin)

# --- Core Communication Logic (Modified for USB) ---
    def _execute_transaction(self, specific_message_payload):
        """
        Sends the protobuf request over USB as a 64-byte framed packet 
        and receives the 64-byte echoed response.
        """
        if self.dev is None or self.ep_out is None or self.ep_in is None:
            raise DeviceConnectionError("USB connection is not established.")

        app_message = self.pb.RequestMessage()
        
        # Determine the oneof field name for the message (Unchanged from serial version)
        field_name = None
        for field in app_message.DESCRIPTOR.fields:
            if field.containing_oneof and field.message_type == specific_message_payload.DESCRIPTOR:
                field_name = field.name
                break
        
        if not field_name:
            raise ValueError(f"Could not find a field in RequestMessage for message type: {type(specific_message_payload).__name__}.")
        
        getattr(app_message, field_name).CopyFrom(specific_message_payload)
        
        request_bytes = app_message.SerializeToString()
        length = len(request_bytes)
        
        # --- 1. SENDING (Fixed 64-byte Packet) ---
        MAX_PAYLOAD_SIZE = self.packet_size - 1 # 63 bytes
        
        if length > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Message too large for {self.packet_size}-byte packet: {length} bytes. Max payload is {MAX_PAYLOAD_SIZE} bytes.")

        # 1. Length prefix (1 byte, little endian is implicit for simple byte, but struct pack ensures 1 byte)
        length_prefix = struct.pack("B", length) 
        
        # 2. Data payload: request_bytes
        
        # 3. Calculate and create padding (zeros)
        padding_needed = self.packet_size - (1 + length)
        padding = b'\x00' * padding_needed
        
        # 4. Build the final 64-byte packet
        packet_to_send = length_prefix + request_bytes + padding

        try:
            # Send the fixed-size packet over the OUT endpoint
            # The pyusb example uses ep_out.write, which handles the transfer.
            self.ep_out.write(packet_to_send)

            # --- 2. RECEIVING (Fixed 64-byte Packet) ---
            # Read the entire fixed-size packet (64 bytes) from the IN endpoint
            full_response_packet = self.ep_in.read(self.packet_size, timeout=self.timeout_ms)
            full_response_packet_bytes = bytes(full_response_packet)
            
        except usb.core.USBError as e:
            raise DeviceResponseError(f"USB Transfer Error: {e}") from e
        
        if len(full_response_packet_bytes) != self.packet_size:
            if len(full_response_packet_bytes) == 0:
                # Timeout occurred
                return self.pb.ResponseMessage() 
            else:
                raise DeviceResponseError(f"Incomplete USB response. Expected {self.packet_size} bytes, got {len(full_response_packet_bytes)}.")

        # Read response length from the first byte
        response_length = full_response_packet_bytes[0]
        
        # Extract the actual payload bytes using the length prefix
        response_bytes = full_response_packet_bytes[1 : 1 + response_length]
        
        # Parse response
        try:
            parsed_response = self.pb.ResponseMessage()
            parsed_response.ParseFromString(response_bytes)
            return parsed_response
        except Exception as e:
            raise DeviceFirmwareError(f"Client-side parse error: {e}. Raw data: {response_bytes.hex()}")

    def _send_and_parse(self, request_payload, expected_response_field: str):
        response_app_msg = self._execute_transaction(request_payload)
        response_field = response_app_msg.WhichOneof("kind")
        if response_field == "error_response":
             raise DeviceFirmwareError(f"{response_app_msg.error_response.message}")
        if response_field != expected_response_field:
             raise DeviceResponseError(expected=expected_response_field, received=response_field)
        return getattr(response_app_msg, response_field)

    def close(self):
        """Cleans up the USB connection and re-attaches the kernel driver if necessary."""
        if self.dev is not None:
            try:
                # Release the interface
                usb.util.release_interface(self.dev, self.interface)
            except usb.core.USBError:
                pass 

            if self.kernel_driver_detached:
                try:
                    self.dev.attach_kernel_driver(self.interface)
                except usb.core.USBError:
                    pass

            usb.util.dispose_resources(self.dev)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # --- High-Level API Methods ---
    def read_firmware_info(self) -> 'ProtobufModule.FirmwareInfoResponse': # Type hint updated
        # Now uses self.pb
        request = self.pb.FirmwareInfoRequest()
        return self._send_and_parse(request, "firmware_info_response")

    def write_bootloader_request(self):
        # Now uses self.pb
        request = self.pb.UsbBootloaderRequest(val=1)
        self._send_and_parse(request, "usb_bootloader_response")

    def read_pin_function(self, gpio_pin):
        # Now uses self.pb
        request = self.pb.GPIOReadFunctionRequest(gpio_pin=gpio_pin)
        return self._send_and_parse(request, "gpio_read_function_response")