
import struct
from enum import Enum
import time

class AP3377Error(Exception):
    """Custom exception for AP3377 operations."""
    pass

class AP33772S:
    """
    Python class for the AP33772S USB-PD Controller IC.
    Translates register operations and data structures from the Zig implementation.
    """
    # I2C Address for the AP33772S (Datasheet Page 16)
    DEVICE_ADDRESS = 0x52

    class Register(Enum):
        """Register Addresses (Commands) from Datasheet Table 16."""
        STATUS = 0x01
        MASK = 0x02
        OPMODE = 0x03
        CONFIG = 0x04
        PDCONFIG = 0x05
        SYSTEM = 0x06
        TR25 = 0x0C
        TR50 = 0x0D
        TR75 = 0x0E
        TR100 = 0x0F
        VOLTAGE = 0x11 # VOUT Voltage, LSB 80mV (2 bytes, LE)
        CURRENT = 0x12 # VOUT Current, LSB 24mA (1 byte)
        TEMP = 0x13    # Temperature, Unit: °C (1 byte)
        VREQ = 0x14    # Requested Voltage, LSB 50mV (2 bytes, LE)
        IREQ = 0x15    # Requested Current, LSB 10mA (2 bytes, LE)
        VSELMIN = 0x16 # Min Selection Voltage, LSB 200mV
        UVPTHR = 0x17  # UVP threshold %
        OVPTHR = 0x18  # OVP threshold offset, LSB 80mV
        OCPTHR = 0x19  # OCP threshold, LSB 50mA
        OTPTHR = 0x1A  # OTP threshold, Unit: °C
        DRTHR = 0x1B   # De-rating threshold, Unit: °C
        SRCPDO = 0x20  # Get All Source PDOs (26 bytes)
        SRC_SPR_PDO1 = 0x21
        SRC_SPR_PDO2 = 0x22
        SRC_SPR_PDO3 = 0x23
        SRC_SPR_PDO4 = 0x24
        SRC_SPR_PDO5 = 0x25
        SRC_SPR_PDO6 = 0x26
        SRC_SPR_PDO7 = 0x27
        SRC_EPR_PDO8 = 0x28
        SRC_EPR_PDO9 = 0x29
        SRC_EPR_PD010 = 0x2A
        SRC_EPR_PDO11 = 0x2B
        SRC_EPR_PDO12 = 0x2C
        SRC_EPR_PDO13 = 0x2D
        PD_REQMSG = 0x31 # Send PDO Request (2 bytes data)
        PD_CMDMSG = 0x32 # Send specific PD command
        PD_MSGRLT = 0x33 # Result of PD request/command (1 byte)

    # --- Data Structures (Bitfields) ---

    class Status:
        """Represents the STATUS register (0x01)."""
        def __init__(self, status_byte: int):
            self.started = bool(status_byte & 0x01)
            self.ready = bool(status_byte & 0x02)
            self.new_pdo = bool(status_byte & 0x04)
            self.under_voltage_protection = bool(status_byte & 0x08)
            self.over_voltage_protection = bool(status_byte & 0x10)
            self.over_current_protection = bool(status_byte & 0x20)
            self.over_temperature_protection = bool(status_byte & 0x40)

    class Config:
        """Represents the CONFIG register (0x04)."""
        def __init__(self, config_byte: int = 0):
            # The Zig definition has bit indices 0-7, which map to the bits below
            # _reserved3 (b7), _reserved2 (b6), _reserved1 (b5)
            self.UVP_EN = bool(config_byte & 0x10) # b4
            self.OVP_EN = bool(config_byte & 0x08) # b3
            self.OCP_EN = bool(config_byte & 0x04) # b2
            self.OTP_EN = bool(config_byte & 0x02) # b1
            self.DR_EN = bool(config_byte & 0x01)  # b0

        def to_byte(self) -> int:
            """Convert the config fields back into a single byte."""
            byte = 0
            if self.DR_EN: byte |= 0x01
            if self.OTP_EN: byte |= 0x02
            if self.OCP_EN: byte |= 0x04
            if self.OVP_EN: byte |= 0x08
            if self.UVP_EN: byte |= 0x10
            # Note: Reserved bits (0xE0) are kept 0 as per DEFAULT_CONFIG
            return byte

    # Default configuration from the Zig code
    DEFAULT_CONFIG = Config(config_byte=0x1E) # DR_EN=0, others=1 -> 0b00011110 = 0x1E

    class PDORequest:
        """Represents the data for a PD Request Message (0x31)."""
        def __init__(self, pdo_index: int, current_select: int, voltage_select: int):
            # pdo_index: u4 (Bits 12:15)
            # current_select: u4 (Bits 8:11)
            # voltage_select: u8 (Bits 0:7)
            if not 0 <= pdo_index <= 0xF: raise ValueError("pdo_index out of range (0-15)")
            if not 0 <= current_select <= 0xF: raise ValueError("current_select out of range (0-15)")
            if not 0 <= voltage_select <= 0xFF: raise ValueError("voltage_select out of range (0-255)")

            self.pdo_index = pdo_index
            self.current_select = current_select
            self.voltage_select = voltage_select

        def to_bytes(self) -> bytes:
            """Packs the request into two little-endian bytes."""
            # Byte 0: VOLTAGE_SEL (Bits 0-7)
            byte0 = self.voltage_select
            # Byte 1: CURRENT_SEL (Bits 8-11) | PDO_INDEX (Bits 12-15)
            byte1 = (self.current_select & 0xF) | ((self.pdo_index & 0xF) << 4)
            return bytes([byte0, byte1])

    class VoltageMin(Enum):
        Reserved = 0
        MinThreshold1 = 1
        MinThreshold2 = 2
        Others = 3

    class SRC_PDO:
        """Represents a Source PDO (2 bytes, Little Endian)."""
        def __init__(self, index: int, pdo_bytes: bytes):
            # Data is Little Endian (LE)
            raw_pdo = struct.unpack('<H', pdo_bytes)[0]

            self.index = index
            self.voltage_max = (raw_pdo & 0x00FF)       # Bits 7:0 (u8)
            self.peak_current_or_voltage_min = (raw_pdo >> 8) & 0x03 # Bits 9:8 (u2)
            self.current_max_code = (raw_pdo >> 10) & 0x0F    # Bits 13:10 (u4)
            self.type = (raw_pdo >> 14) & 0x01          # Bit 14 (u1)
            self.detect = (raw_pdo >> 15) & 0x01        # Bit 15 (u1)

        def get_voltage_mv(self, is_epr: bool) -> int:
            """Calculates VOUT voltage in mV."""
            # Scaling: 100mV/LSB for SPR/PPS, 200mV/LSB for EPR/AVS
            scale_mv = 200 if is_epr and self.type == 0 else 100 # Adjusted logic based on typical PD spec
            return self.voltage_max * scale_mv

        def get_current_ma(self) -> int:
            """Calculates VOUT current in mA."""
            # Formula from Zig: 1000 + (current_max_code * 266)
            return 1000 + (self.current_max_code * 266)

        def is_pps(self) -> bool:
            return self.type == 1

        def get_voltage_min_mv(self, is_epr: bool) -> int:
            """Calculates minimum voltage for PPS/AVS ranges."""
            try:
                voltage_min_code = AP33772S.VoltageMin(self.peak_current_or_voltage_min)
            except ValueError:
                return 0 # Unknown code

            if voltage_min_code == AP33772S.VoltageMin.MinThreshold1:
                return 15000 if is_epr else 3300
            elif voltage_min_code == AP33772S.VoltageMin.MinThreshold2:
                # Assuming this represents the lower bound of a range
                return 15000 if is_epr else 5000
            else:
                return 0

    # --- Initialization ---

    def __init__(self, i2c_instance):
        """
        Initializes the AP33772S device object.
        i2c_instance: An object matching the LogicWeave i2c interface.
        """
        self.i2c_instance = i2c_instance
        self.configure_protections(self.DEFAULT_CONFIG)

    def __del__(self):
        """Called upon object deletion (approximation of deinit)."""
        # In Python, this is not guaranteed, but good practice
        # self.deinit() # Actual deinit logic not strictly necessary in Python I2C context

    # --- Private Helper Functions ---

    def _write_register(self, reg_addr: Register, data: bytes) -> None:
        """
        Writes data to a specific register/command.
        I2C format: [Command Byte] [Data Bytes...]
        """
        if not isinstance(data, bytes):
            raise TypeError("Data must be a bytes object.")

        # Command Byte is the first byte of the data sent
        write_buffer = bytes([reg_addr.value]) + data

        try:
            # I2C Write: [Device Address + W] [Command Byte] [Data Bytes...] [Stop]
            self.i2c_instance.write(device_address=self.DEVICE_ADDRESS, data=write_buffer)
        except Exception as e:
            raise AP3377Error(f"I2C write error on register {reg_addr.name}: {e}")

    def _read_register(self, reg_addr: Register, byte_count: int) -> bytes:
        """
        Reads data from a specific register/command.
        I2C format: [Device Address + W] [Command Byte] [Restart] [Device Address + R] [Read Data Bytes...] [Stop]
        """
        try:
            # Step 2: Read the data back (implicitly repeated start for some libraries)
            resp = self.i2c_instance.write_then_read(device_address=self.DEVICE_ADDRESS, data=bytes([reg_addr.value]), byte_count=byte_count)
            
            if len(resp) != byte_count:
                 raise AP3377Error(f"Expected {byte_count} bytes, received {len(resp)} for {reg_addr.name}.")

            return resp
        except Exception as e:
            raise AP3377Error(f"I2C read error on register {reg_addr.name}: {e}")

    # --- Public API Functions ---

    def get_status(self) -> Status:
        """Reads and decodes the STATUS register (0x01)."""
        status_byte = self._read_register(self.Register.STATUS, 1)[0]
        return self.Status(status_byte)

    def get_operation_mode(self) -> int:
        """Reads the OPMODE register (0x03)."""
        opmode_byte = self._read_register(self.Register.OPMODE, 1)[0]
        return opmode_byte

    def read_voltage_mv(self) -> int:
        """Reads VOUT voltage (0x11) and converts to mV. LSB is 80mV (2 bytes, LE)."""
        data = self._read_register(self.Register.VOLTAGE, 2)
        raw_voltage = struct.unpack('<H', data)[0] # Little Endian (H for u16)
        return raw_voltage * 80

    def read_current_ma(self) -> int:
        """Reads VOUT current (0x12) and converts to mA. LSB is 24mA (1 byte)."""
        data = self._read_register(self.Register.CURRENT, 1)
        raw_current = data[0]
        return raw_current * 24

    def read_temperature(self) -> int:
        """Reads the TEMP register (0x13). Unit: °C (1 byte)."""
        data = self._read_register(self.Register.TEMP, 1)
        return data[0]

    def read_requested_voltage_mv(self) -> int:
        """Reads requested voltage (VREQ, 0x14) and converts to mV. LSB is 50mV (2 bytes, LE)."""
        data = self._read_register(self.Register.VREQ, 2)
        raw_vreq = struct.unpack('<H', data)[0]
        return raw_vreq * 50

    def read_requested_current_ma(self) -> int:
        """Reads requested current (IREQ, 0x15) and converts to mA. LSB is 10mA (2 bytes, LE)."""
        data = self._read_register(self.Register.IREQ, 2)
        raw_ireq = struct.unpack('<H', data)[0]
        return raw_ireq * 10

    def request_pdo(self, pdo_request: PDORequest, timeout_s: float = 1.0) -> bool:
            """
            Sends a PDO request message (PD_REQMSG 0x31) and polls the result.

            Args:
                pdo_request: The PDORequest object to send.
                timeout_s: Maximum time in seconds to wait for a successful response (PD_MSGRLT = 1).

            Returns:
                True if the negotiation was successful (PD_MSGRLT = 1), False otherwise.
            """
            request_bytes = pdo_request.to_bytes()
            
            # 1. Send the request
            self._write_register(self.Register.PD_REQMSG, request_bytes)

            # 2. Poll the PD_MSGRLT register
            start_time = time.monotonic()
            polling_interval = 0.01  # Poll frequently (10ms)

            while time.monotonic() - start_time < timeout_s:
                result = self.get_pd_message_result()
                
                # PD_MSGRLT = 1 means negotiation successful (PS_RDY received)
                if result == 1:
                    # Clear the result bit by writing the command again to reset for next negotiation (common I2C register pattern)
                    # Or based on the flowchart, this read/check is where the MCU determines success
                    return True
                
                # If a fault is signaled (e.g., 0x02 for Hard Reset or another error if defined, though datasheet only mentions 1)
                # you may want to handle it here. For now, we only check for success (1).
                if result != 0:
                    # If result is not 0 or 1, it might be an unhandled error code (e.g., 2 for Hard Reset result)
                    print(f"PDO request failed with result code: {result}")
                    return False

                time.sleep(polling_interval)
            
            # 3. Timeout reached
            print(f"PDO request timed out after {timeout_s} seconds. Result was 0.")
            return False

    def get_pd_message_result(self) -> int:
        """Reads the result of the last PD request/command (PD_MSGRLT 0x33)."""
        data = self._read_register(self.Register.PD_MSGRLT, 1)
        return data[0]

    def set_otp_threshold(self, threshold_celsius: int) -> None:
        """Sets OTP Threshold (OTPTHR 0x1A)."""
        if not 0 <= threshold_celsius <= 255: raise ValueError("Threshold must be 0-255.")
        self._write_register(self.Register.OTPTHR, bytes([threshold_celsius]))

    def configure_protections(self, config: Config) -> None:
        """Configures protection features (CONFIG 0x04)."""
        config_byte = config.to_byte()
        self._write_register(self.Register.CONFIG, bytes([config_byte]))

    def set_pd_config(self, config_byte: int) -> None:
        """Sets PD Config (PDCONFIG 0x05)."""
        self._write_register(self.Register.PDCONFIG, bytes([config_byte]))

    def read_all_source_pdos(self) -> bytes:
        """Reads all source PDOs (SRCPDO 0x20, 26 bytes)."""
        return self._read_register(self.Register.SRCPDO, 26)

    def read_source_pdo(self, index: int, is_epr: bool) -> SRC_PDO:
        """Reads a specific Source PDO (index 1-13) and returns a parsed SRC_PDO object."""
        if not 1 <= index <= 13:
            raise AP3377Error(f"Invalid PDO index: {index}. Must be between 1 and 13.")

        # Calculate the register address (0x20 + index)
        register_addr_val = self.Register.SRC_SPR_PDO1.value + (index - 1)
        
        # Safe way to get the enum member
        try:
            register_addr = self.Register(register_addr_val)
        except ValueError:
            raise AP3377Error(f"Calculated register address 0x{register_addr_val:02X} is not a valid PDO register.")

        pdo_bytes = self._read_register(register_addr, 2)
        
        # The is_epr check is passed for PDO calculations but not used in the read itself
        return self.SRC_PDO(index, pdo_bytes)