import time
import platform
from typing import Protocol, Any

# --- CONSTANTS ---
# Kept here because the Transport layer needs them to find the device.
VENDOR_ID = 0x2E8A
PRODUCT_ID = 0x000a
INTERFACE_NUM = 1
EP_IN_ADDR = 0x84
EP_OUT_ADDR = 0x05
PACKET_SIZE = 64

class Transport(Protocol):
    def open(self) -> None: ...
    def write(self, data: bytes) -> None: ...
    def read(self, length: int, timeout_ms: int) -> bytes: ...
    def close(self) -> None: ...

class NativeUsbTransport:
    """Implementation using PyUSB for running locally on Windows/Linux/Mac."""
    def __init__(self, vendor_id=VENDOR_ID, product_id=PRODUCT_ID, interface=0):
        # Local imports so this class doesn't crash on the web
        import usb.core
        import usb.util
        self.usb_core = usb.core
        self.usb_util = usb.util
        self.platform = platform
        
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.interface = interface
        self.dev = None
        self.ep_out = None
        self.ep_in = None
        self.kernel_driver_detached = False

    def open(self):
        self.dev = self.usb_core.find(idVendor=self.vendor_id, idProduct=self.product_id)
        if self.dev is None:
            raise Exception("Device not found")

        # Kernel driver detachment logic
        if self.platform.system() != "Windows":
            try:
                if self.dev.is_kernel_driver_active(self.interface):
                    self.dev.detach_kernel_driver(self.interface)
                    self.kernel_driver_detached = True
            except (NotImplementedError, self.usb_core.USBError):
                pass

        self.dev.set_configuration()
        cfg = self.dev.get_active_configuration()
        intf = cfg[(self.interface, 0)]

        self.ep_out = self.usb_util.find_descriptor(
            intf,
            custom_match=lambda e: e.bEndpointAddress == EP_OUT_ADDR
        )
        
        self.ep_in = self.usb_util.find_descriptor(
            intf,
            custom_match=lambda e: e.bEndpointAddress == EP_IN_ADDR
        )

        if self.ep_out is None:
             raise Exception(f"Endpoint OUT (0x{EP_OUT_ADDR:02x}) not found")
        if self.ep_in is None:
             raise Exception(f"Endpoint IN (0x{EP_IN_ADDR:02x}) not found")

    def write(self, data: bytes):
        if not self.ep_out: raise Exception("Not connected")
        self.ep_out.write(data)

    def read(self, length: int, timeout_ms: int) -> bytes:
        if not self.ep_in: raise Exception("Not connected")
        return bytes(self.ep_in.read(length, timeout=timeout_ms))

    def close(self):
        if self.dev:
            try:
                self.usb_util.release_interface(self.dev, self.interface)
                if self.kernel_driver_detached:
                    self.dev.attach_kernel_driver(self.interface)
                self.usb_util.dispose_resources(self.dev)
            except:
                pass

class PyodideTransport:
    """
    WebUSB Transport for running inside the Browser via Pyodide.
    Uses pyodide.ffi.run_sync to bridge async JS calls to sync Python.
    """
    def __init__(self, vendor_id=VENDOR_ID, product_id=PRODUCT_ID, interface=INTERFACE_NUM):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.interface = interface
        self.js_dev = None

    def open(self):
        import js
        from pyodide.ffi import run_sync
        
        # 1. Check if worker.js set the device globally
        if not hasattr(js, 'py_device') or not js.py_device:
            raise Exception("No authorized USB device found in browser context.")
        
        # 2. Define the async JS steps
        async def _async_open():
            dev = js.py_device
            await dev.open()
            await dev.selectConfiguration(1)
            await dev.claimInterface(self.interface)
            return dev

        # 3. Execute synchronously
        try:
            self.js_dev = run_sync(_async_open())
        except Exception as e:
            raise Exception(f"Failed to open WebUSB device: {e}")

    def write(self, data: bytes):
        if not self.js_dev:
            raise Exception("Device not open")
        
        import js
        from pyodide.ffi import run_sync

        # Convert Python bytes to JS Uint8Array
        js_data = js.Uint8Array.new(data)

        async def _async_write():
            # WebUSB transferOut(endpointNumber, data)
            # EP_OUT_ADDR is 0x05, which is valid for transferOut
            await self.js_dev.transferOut(EP_OUT_ADDR, js_data)

        run_sync(_async_write())

    def read(self, length: int, timeout_ms: int) -> bytes:
        if not self.js_dev:
            raise Exception("Device not open")

        from pyodide.ffi import run_sync

        async def _async_read():
            # EP_IN_ADDR is 0x84. WebUSB expects the Endpoint Number (4).
            # We strip the direction bit (0x80) just in case, though some browsers handle 0x84.
            endpoint_num = EP_IN_ADDR & 0x0F 
            
            result = await self.js_dev.transferIn(endpoint_num, length)
            # result.data is a DataView. buffer.to_bytes() converts it back to Python.
            return result.data.buffer.to_bytes()

        return run_sync(_async_read())

    def close(self):
        if not self.js_dev:
            return

        from pyodide.ffi import run_sync

        async def _async_close():
            try:
                await self.js_dev.close()
            except:
                pass 

        run_sync(_async_close())
        self.js_dev = None