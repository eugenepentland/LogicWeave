# exceptions.py

class DeviceError(Exception):
    """Base exception class for all device-related errors."""
    pass

class DeviceConnectionError(DeviceError):
    """Raised for errors related to the serial port connection."""
    pass

class DeviceFirmwareError(DeviceError):
    """
    Raised when the device's firmware returns an explicit error message.
    This corresponds to receiving an `ErrorResponse` protobuf message.
    """
    def __init__(self, message):
        self.message = message
        super().__init__(f"Firmware error: {message}")

class DeviceResponseError(DeviceError):
    """
    Raised when the device gives a response that was not expected for a given request.
    For example, if you send a GPIOReadRequest but do not get a GPIOReadResponse.
    """
    def __init__(self, expected, received):
        self.expected = expected
        self.received = received
        super().__init__(f"Unexpected response. Expected a response containing '{expected}', but got '{received}'.")

