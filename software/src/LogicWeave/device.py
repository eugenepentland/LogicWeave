import serial
import time
import struct # Required for packing/unpacking length prefixes
import LogicWeave.proto_gen.all_pb2 as all_pb2 # Changed import to the standard protobuf generated file
from LogicWeave.exceptions import DeviceFirmwareError, DeviceResponseError, DeviceConnectionError
import serial.tools.list_ports

def _get_device_port():
    vid = 0x1E8B
    pid = 0x0001
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

class LogicWeave:
    """
    A self-contained, high-level wrapper for communicating with the device over
    a serial connection using the defined protobuf messages. This class handles
    all serial communication and protobuf message handling internally.

    Includes a 'file' mode to log serialized commands to a file instead of
    sending them over serial.
    """

    def __init__(self, port=None, baudrate=115200, timeout=1, write_delay=0, mode="serial", output_file=None, **kwargs):
        """
        Initializes the Device and establishes a serial connection or prepares for file logging.

        Args:
            port (str, optional): The serial port (e.g., 'COM3' or '/dev/ttyUSB0').
                                  Required if mode is "serial".
            baudrate (int): The serial communication speed.
            timeout (int): The read timeout for the serial connection in seconds.
            write_delay (float): The delay in seconds after a write operation (only for serial mode).
            mode (str): The operational mode: "serial" for actual communication,
                        "file" to write commands to a file. Defaults to "serial".
            output_file (str, optional): The path to the file where commands will be logged
                                         if mode is "file". Required if mode is "file".
            **kwargs: Additional arguments for pyserial's Serial constructor.
        """
        self.mode = mode
        self.output_file = output_file
        self.ser = None
        self.file_handle = None  # New attribute for file handle when in 'file' mode
        self.write_delay = write_delay

        if self.mode == "serial":
            if not port:
                port = _get_device_port()
                if not port:
                    raise ValueError("Port must be specified when mode is 'serial'.")
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=timeout,
                    **kwargs
                )
            except serial.SerialException as e:
                raise DeviceConnectionError(f"Failed to connect to {port}: {e}") from e
        elif self.mode == "file":
            if not self.output_file:
                raise ValueError("output_file must be specified when mode is 'file'.")
            try:
                # Open in append binary mode, so multiple commands can be logged
                self.file_handle = open(self.output_file, "ab")
            except IOError as e:
                raise DeviceConnectionError(f"Failed to open output file {self.output_file}: {e}") from e
        else:
            raise ValueError(f"Invalid mode: {mode}. Must be 'serial' or 'file'.")

        # This map is used to construct the AppMessage for sending.
        # Keys are the request message classes, values are the corresponding
        # field names in the AppMessage 'kind' oneof group.
        # Note: With regular protobuf, you directly assign to the oneof field.
        self._request_to_field_map = {
            all_pb2.GPIOModeRequest: "gpio_mode_request",
            all_pb2.GPIOWriteRequest: "gpio_write_request",
            all_pb2.UsbBootloaderRequest: "usb_bootloader_request",
            all_pb2.GPIOReadRequest: "gpio_read_request",
            all_pb2.EchoMessage: "echo_message",
            all_pb2.I2CSetupRequest: "i2c_setup_request",
            all_pb2.I2CWriteRequest: "i2c_write_request",
            all_pb2.I2CReadRequest: "i2c_read_request",
            all_pb2.SPISetupRequest: "spi_setup_request",
            all_pb2.SPIReadRequest: "spi_read_request",
            all_pb2.SPIWriteRequest: "spi_write_request",
            all_pb2.UsbPDReadRequest: "usb_pd_read_request",
            all_pb2.UsbPDReadPDORequest: "usb_pd_read_pdo_request",
            all_pb2.UsbPDWritePDORequest: "usb_pd_write_pdo_request",
            all_pb2.WriteTextRequest: "write_text_request",
            all_pb2.UsbPDEnableRequest: "usb_pd_enable_request",
            all_pb2.ClearScreenRequest: "clear_screen_request",
            all_pb2.RefreshScreenRequest: "refresh_screen_request",
            all_pb2.FirmwareInfoRequest: "firmware_info_request",
            all_pb2.SoftSPIWriteRequest: "soft_spi_write_request",
            all_pb2.WriteBankVoltageRequest: "write_bank_voltage_request"
        }

        # Map for response field names to their corresponding message classes.
        # Used to create dummy responses when in 'file' mode.
        self._response_field_to_class_map = {
            "firmware_info_response": all_pb2.FirmwareInfoResponse,
            "echo_message": all_pb2.EchoMessage, # EchoMessage can be both request and response
            "gpio_read_response": all_pb2.GPIOReadResponse,
            "i2c_read_response": all_pb2.I2CReadResponse,
            "spi_read_response": all_pb2.SPIReadResponse,
            "usb_pd_read_response": all_pb2.UsbPDReadResponse,
            "usb_pd_read_pdo_response": all_pb2.UsbPDReadPDOResponse,
            # For responses that are typically just Empty messages (assuming 'Empty' exists in messages_pb2.py)
            "empty": all_pb2.Empty,
            "gpio_write_response": all_pb2.Empty,
            "i2c_setup_response": all_pb2.Empty,
            "i2c_write_response": all_pb2.Empty,
            "spi_setup_response": all_pb2.Empty,
            "spi_write_response": all_pb2.Empty,
            "soft_spi_write_response": all_pb2.Empty,
            "usb_pd_enable_response": all_pb2.Empty,
            "usb_pd_write_pdo_response": all_pb2.Empty,
            "gpio_mode_response": all_pb2.Empty,
            "write_text_response": all_pb2.Empty,
            "clear_screen_response": all_pb2.Empty,
            "refresh_screen_response": all_pb2.Empty,
            "usb_bootloader_response": all_pb2.Empty,
            "write_bank_voltage_response": all_pb2.Empty,
            "error_response": all_pb2.ErrorResponse, # For device-side error messages
        }

    def _execute_transaction(self, specific_message_payload) -> all_pb2.AppMessage | None:
        """
        Handles the full send-and-receive cycle for a protobuf message.
        It wraps the payload, sends it (serial mode), and returns the parsed AppMessage response.
        In 'file' mode, it writes the length-prefixed message to the file and returns None.

        Args:
            specific_message_payload (google.protobuf.message.Message): The specific protobuf
                                                                         request message to send.

        Returns:
            messages_pb2.AppMessage: The parsed AppMessage response received from the device (serial mode).
            None: If operating in 'file' mode (no actual response from a device).

        Raises:
            DeviceConnectionError: If the serial port/output file is not open.
            ValueError: If the specific_message_payload type is not supported
                        or if message is too large for the 2-byte prefix.
            DeviceFirmwareError: If there's an error parsing the response bytes (serial mode).
            DeviceResponseError: If an incomplete response is received (serial mode).
        """
        message_type = type(specific_message_payload)
        field_name = self._request_to_field_map.get(message_type)

        if not field_name:
            raise ValueError(f"Unsupported message type: {message_type.__name__}")

        # Wrap the specific payload into the main AppMessage container
        app_message_request = all_pb2.AppMessage()
        
        # --- THE CRUCIAL CHANGE FOR REGULAR PROTOBUF ONEOF ASSIGNMENT ---
        # 1. Get the accessor for the oneof field (which returns a sub-message object).
        # 2. Use CopyFrom to populate that sub-message object with your specific payload.
        getattr(app_message_request, field_name).CopyFrom(specific_message_payload)
        
        # Serialize the AppMessage request
        request_bytes = app_message_request.SerializeToString() # Use regular protobuf serialization
        length = len(request_bytes)

        # Ensure the message length fits within a 2-byte unsigned integer (max 65535)
        if length > 65535:
            raise ValueError(f"Message too large for 2-byte prefix: {length} bytes. Max is 65535.")

        # Pack the length into 2 bytes, big-endian format
        length_prefix = struct.pack(">H", length) # '>H' means big-endian unsigned short

        if self.mode == "serial":
            if not self.ser or not self.ser.is_open:
                raise DeviceConnectionError("Serial port is not open.")

            # Clear input buffer to ensure we only read the response to this request
            self.ser.reset_input_buffer()
            
            # Send the 2-byte length prefix followed by the serialized request bytes
            self.ser.write(length_prefix + request_bytes)

            # This delay gives the device time to process the request and begin its response.
            if self.write_delay > 0:
                time.sleep(self.write_delay)

            # --- Read Response (assuming device sends a 1-byte length prefix for its responses) ---
            # Read the 1-byte length prefix of the response from the device
            response_length_byte = self.ser.read(1)
            if not response_length_byte:
                # If no length byte is received within timeout, return an empty AppMessage
                # This could indicate a timeout or a device that doesn't always respond.
                return all_pb2.AppMessage() # Return an empty AppMessage instance
            
            response_length = response_length_byte[0] # Convert the single byte to an integer

            # Read the actual response message bytes based on the received length
            response_bytes = self.ser.read(response_length)
            
            if len(response_bytes) != response_length:
                # This indicates an incomplete read, possibly due to timeout or data corruption
                raise DeviceResponseError(f"Incomplete response received. Expected {response_length} bytes, got {len(response_bytes)}.")

            try:
                # Parse the received response bytes into an AppMessage
                parsed_response = all_pb2.AppMessage()
                parsed_response.ParseFromString(response_bytes) # Use regular protobuf deserialization
                return parsed_response
            except Exception as e:
                # If protobuf parsing fails, wrap it in a firmware error for clarity
                raise DeviceFirmwareError(f"Client-side parse error: {e}. Raw data: {response_bytes.hex()}")

        elif self.mode == "file":
            if not self.file_handle:
                raise DeviceConnectionError("Output file is not open.")
            
            # Write the 2-byte length prefix to the file
            self.file_handle.write(length_prefix)
            # Write the serialized request bytes to the file
            self.file_handle.write(request_bytes)
            
            # In file mode, there is no actual device response, so return None
            return None

    def _send_and_parse(self, request_payload, expected_response_field: str):
        """
        A wrapper around _execute_transaction that validates the response type
        and handles generic error responses from the device (serial mode)
        or generates a dummy response (file mode).

        Args:
            request_payload (google.protobuf.message.Message): The specific protobuf
                                                                 request message to send.
            expected_response_field (str): The name of the expected field in
                                           the AppMessage 'kind' oneof group
                                           for a successful response.

        Returns:
            google.protobuf.message.Message: The specific protobuf response message payload.

        Raises:
            DeviceFirmwareError: If the device returns an error response (serial mode).
            DeviceResponseError: If the received response type does not match
                                 the expected type (serial mode).
            RuntimeError: If a dummy response cannot be generated in file mode
                          (should not happen if _response_field_to_class_map is complete).
        """
        response_app_msg = self._execute_transaction(request_payload)

        if self.mode == "file":
            # If in 'file' mode, _execute_transaction returns None, as there's no actual response.
            # We then return a default-constructed instance of the expected response type
            # to maintain the high-level API's return type consistency.
            response_class = self._response_field_to_class_map.get(expected_response_field)
            if response_class:
                return response_class() # Return an empty/default instance of the expected response message
            else:
                # This case indicates an unmapped response field. It should ideally not occur.
                raise RuntimeError(f"Cannot generate dummy response for unknown type: '{expected_response_field}' in file mode.")

        # --- Original serial mode logic continues below ---
        # With regular protobuf, to determine which oneof field is set, you use HasField
        # and then directly access the field.
        response_field = response_app_msg.WhichOneof("kind") # "kind" is the name of the oneof group

        # Check for a generic error response from the device
        if response_field == "error_response":
            raise DeviceFirmwareError(f"Device error: {response_app_msg.error_response.message}")
        
        # Validate that the received response type matches the expected type
        if response_field != expected_response_field:
            raise DeviceResponseError(expected=expected_response_field, received=response_field)

        # Access the payload directly using the determined field name
        response_payload = getattr(response_app_msg, response_field)
        return response_payload

    def close(self):
        """Closes the serial connection or output file if open."""
        if self.mode == "serial" and self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")
        elif self.mode == "file" and self.file_handle:
            self.file_handle.close()
            self.file_handle = None # Clear the handle
            print(f"Output file '{self.output_file}' closed.")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # --- High-Level API Methods ---

    def read_firmware_info(self) -> all_pb2.FirmwareInfoResponse:
        """
        Reads firmware version and build information from the device.
        In 'file' mode, logs the request and returns a default FirmwareInfoResponse.
        """
        request = all_pb2.FirmwareInfoRequest(info=1)
        response = self._send_and_parse(request, "firmware_info_response")
        return response

    def write_read_echo(self, message: str) -> str:
        """
        Sends a message to the device and returns its echoed response.
        In 'file' mode, logs the request and returns an empty string.
        """
        request = all_pb2.EchoMessage(message=message)
        response = self._send_and_parse(request, "echo_message")
        return response.message if self.mode == "serial" else "" # Return empty string for file mode

    # --- GPIO Methods ---
    
    def write_gpio_mode(self, pin: int, mode: all_pb2.Mode):
        """
        Writes the mode (input, output, pwm) for a specific GPIO pin.
        In 'file' mode, logs the request.
        """
        request = all_pb2.GPIOModeRequest(gpio_pin=pin, mode=mode)
        self._send_and_parse(request, "gpio_mode_response") # Expects an Empty response

    def gpio_write(self, pin: int, state: bool):
        """
        Writes a boolean state (high/low) to a GPIO pin.
        In 'file' mode, logs the request.
        """
        request = all_pb2.GPIOWriteRequest(gpio_pin=pin, state=state)
        self._send_and_parse(request, "gpio_write_response") # Expects a gpio_write_response (Empty)

    def gpio_read(self, pin: int) -> bool:
        """
        Reads the state of a GPIO pin.
        In 'file' mode, logs the request and returns False.
        """
        request = all_pb2.GPIOReadRequest(gpio_pin=pin)
        response = self._send_and_parse(request, "gpio_read_response")
        return response.state if self.mode == "serial" else False # Return default for file mode

    # --- I2C Methods ---

    def write_i2c_setup(self, instance_num: int, sda_pin: int, scl_pin: int):
        """
        Writes the configuration for an I2C peripheral instance on the device.
        In 'file' mode, logs the request.
        """
        request = all_pb2.I2CSetupRequest(
            instance_num=instance_num, 
            sda_pin=sda_pin, 
            scl_pin=scl_pin
        )
        self._send_and_parse(request, "i2c_setup_response")

    def i2c_write(self, instance_num: int, device_address: int, data: bytes):
        """
        Writes data to an I2C device.
        In 'file' mode, logs the request.
        """
        request = all_pb2.I2CWriteRequest(
            instance_num=instance_num,
            device_address=device_address,
            data=data
        )
        self._send_and_parse(request, "i2c_write_response")

    def i2c_read(self, instance_num: int, device_address: int, byte_count: int) -> bytes:
        """
        Reads data from an I2C device.
        In 'file' mode, logs the request and returns an empty bytes object.
        """
        request = all_pb2.I2CReadRequest(
            instance_num=instance_num,
            device_address=device_address,
            byte_count=byte_count
        )
        response = self._send_and_parse(request, "i2c_read_response")
        return response.data if self.mode == "serial" else b"" # Return default for file mode

    # --- SPI Methods ---

    def write_spi_setup(self, instance_num: int, cs_pin: int, sclk_pin: int, mosi_pin: int, miso_pin: int):
        """
        Writes the configuration for an SPI peripheral instance on the device.
        In 'file' mode, logs the request.
        """
        request = all_pb2.SPISetupRequest(
            instance_num=instance_num,
            cs_pin=cs_pin,
            sclk_pin=sclk_pin,
            mosi_pin=mosi_pin,
            miso_pin=miso_pin
        )
        self._send_and_parse(request, "spi_setup_response")

    def spi_write(self, instance_num: int, data: bytes, cs_pin: int):
        """
        Writes data over an SPI interface.
        In 'file' mode, logs the request.
        """
        request = all_pb2.SPIWriteRequest(
            instance_num=instance_num,
            data=data,
            cs_pin=cs_pin
        )
        self._send_and_parse(request, "spi_write_response")

    def soft_spi_write(self, cs_pin: int, sclk_pin: int, mosi_pin: int, data: bytes):
        """
        Writes data over a software SPI interface (bit-banged).
        In 'file' mode, logs the request.
        """
        request = all_pb2.SoftSPIWriteRequest(
            cs_pin=cs_pin,
            sclk_pin=sclk_pin,
            mosi_pin=mosi_pin,
            data=data
        )
        self._send_and_parse(request, "soft_spi_write_response")

    def spi_read(self, instance_num: int, byte_count: int, cs_pin: int) -> bytes:
        """
        Reads data over an SPI interface.
        In 'file' mode, logs the request and returns an empty bytes object.
        """
        request = all_pb2.SPIReadRequest(
            instance_num=instance_num,
            data=b'', # Data field typically not used for outgoing in a pure read
            cs_pin=cs_pin,
            byte_count=byte_count
        )
        response = self._send_and_parse(request, "spi_read_response")
        return response.data if self.mode == "serial" else b"" # Return default for file mode

    # --- USB PD Methods ---

    def write_pd_output_state(self, channel: int, on: bool):
        """
        Writes the enabled/disabled state for a USB PD channel's output.
        In 'file' mode, logs the request.
        """
        request = all_pb2.UsbPDEnableRequest(channel=channel, on=on)
        self._send_and_parse(request, "usb_pd_enable_response")

    def read_pd_channel_status(self, channel: int) -> all_pb2.UsbPDReadResponse:
        """
        Reads the current voltage and current of a USB PD channel.
        In 'file' mode, logs the request and returns a default UsbPDReadResponse.
        """
        request = all_pb2.UsbPDReadRequest(channel=channel)
        response = self._send_and_parse(request, "usb_pd_read_response")
        return response

    def read_pd_source_capabilities(self, channel: int, index: int) -> all_pb2.UsbPDReadPDOResponse:
        """
        Reads a specific Power Data Object (PDO) from a USB PD source,
        describing its power capabilities.
        In 'file' mode, logs the request and returns a default UsbPDReadPDOResponse.
        """
        request = all_pb2.UsbPDReadPDORequest(channel=channel, index=index)
        response = self._send_and_parse(request, "usb_pd_read_pdo_response")
        return response
        
    def write_pd_power_request(self, channel: int, voltage_mv: int, current_limit_ma: int):
        """
        Writes a request for a specific voltage and current from a USB PD source.
        In 'file' mode, logs the request.
        """
        request = all_pb2.UsbPDWritePDORequest(
            channel=channel,
            voltage_mv=voltage_mv,
            current_limit_ma=current_limit_ma
        )
        self._send_and_parse(request, "usb_pd_write_pdo_response")

    def write_bank_voltage(self, bank: int, voltage: all_pb2.BankVoltage):
        """
        Writes the desired voltage for a specific power bank.
        In 'file' mode, logs the request.
        """
        request = all_pb2.WriteBankVoltageRequest(
            bank=bank,
            voltage=voltage
        )
        self._send_and_parse(request, "write_bank_voltage_response")

    # --- Screen Methods ---

    def write_text(self, text: str, x: int = 0, y: int = 0):
        """
        Writes text to the device's screen at a specified position.
        In 'file' mode, logs the request.
        """
        request = all_pb2.WriteTextRequest(text=text, x=x, y=y)
        self._send_and_parse(request, "write_text_response")

    def write_clear_screen(self):
        """
        Writes a command to clear the device's screen.
        In 'file' mode, logs the request.
        """
        request = all_pb2.ClearScreenRequest()
        self._send_and_parse(request, "clear_screen_response")
        
    def write_refresh_screen(self):
        """
        Writes a command to refresh the device's screen to show updates.
        In 'file' mode, logs the request.
        """
        request = all_pb2.RefreshScreenRequest()
        self._send_and_parse(request, "refresh_screen_response") 

    def write_bootloader_request(self):
        """
        Writes a request for the device to enter its USB bootloader mode.
        After this command, the device typically reboots into bootloader and
        the serial connection will likely be lost.
        In 'file' mode, logs the request.
        """
        request = all_pb2.UsbBootloaderRequest(val=1)
        self._send_and_parse(request, "usb_bootloader_response")

# Utility function to read and parse commands from the generated binary file
def read_commands_from_file(file_path: str) -> list[all_pb2.AppMessage]:
    """
    Reads length-prefixed AppMessage commands from a binary file.
    Each message is expected to be prefixed with a 2-byte length.
    """
    commands = []
    with open(file_path, "rb") as f:
        while True:
            # Read the 2-byte length prefix
            length_prefix = f.read(2)
            if not length_prefix:
                break  # End of file

            # Unpack the 2-byte length (unsigned short, big-endian)
            length = struct.unpack(">H", length_prefix)[0]

            # Read the actual serialized AppMessage data
            serialized_data = f.read(length)
            if len(serialized_data) != length:
                raise EOFError(f"Incomplete message data in file '{file_path}'. Expected {length} bytes, got {len(serialized_data)}.")

            try:
                # Parse the data back into an AppMessage
                command_message = all_pb2.AppMessage() # Create an instance
                command_message.ParseFromString(serialized_data) # Parse the bytes into it
                commands.append(command_message)
            except Exception as e:
                print(f"Warning: Could not parse message from file: {e}")
    return commands