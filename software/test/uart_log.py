import serial
import time

# --- Configuration ---
# Replace 'COM3' with your serial port name (e.g., '/dev/ttyUSB0' on Linux, 'COM1' on Windows)
SERIAL_PORT = 'COM17'
BAUD_RATE = 115200  # Common baud rates: 9600, 19200, 38400, 57600, 115200

# --- Main Program ---
def read_and_print_uart():
    """
    Initializes the serial port, reads incoming bytes, and prints them to the console.
    """
    print(f"Attempting to open serial port: {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        # Open the serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully opened {SERIAL_PORT}. Reading data...")
        print("Press Ctrl+C to stop.")

        while True:
            # Read all bytes currently in the receive buffer
            # read() returns bytes, so we decode it to a string for printing
            # You can adjust the number of bytes to read, e.g., ser.read(1) for one byte at a time
            # or ser.readline() for line-by-line reading if your device sends newlines.
            data = ser.read(ser.in_waiting) # Read all available bytes

            if data:
                try:
                    # Attempt to decode the bytes. 'utf-8' is common, but 'latin-1' or 'ascii' might be needed.
                    # 'errors=replace' will replace un-decodable bytes with a replacement character.
                    decoded_data = data.decode('utf-8', errors='replace')
                    print(decoded_data, end='') # Use end='' to avoid extra newlines if data already has them
                except UnicodeDecodeError as e:
                    print(f"UnicodeDecodeError: {e} - Raw bytes: {data}")
                except Exception as e:
                    print(f"An unexpected error occurred during decoding: {e}")
            
            time.sleep(0.01) # Small delay to prevent busy-waiting

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. Please check:")
        print(f"  - Is the port name correct? (e.g., 'COM3' or '/dev/ttyUSB0')")
        print(f"  - Is the device connected?")
        print(f"  - Is the port already in use by another application?")
        print(f"  - Do you have the necessary permissions? (e.g., add user to 'dialout' group on Linux)")
        print(f"  - Original error: {e}")
    except KeyboardInterrupt:
        print("\nExiting program. Serial port closed.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Ensure the serial port is closed if it was opened
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    read_and_print_uart()
