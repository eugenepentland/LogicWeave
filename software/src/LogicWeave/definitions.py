import enum

class GPIOMode(enum.IntEnum):
    INPUT = 0
    OUTPUT = 1
    PWM = 2

class BankVoltage(enum.IntEnum):
    V1P8 = 0
    V3P3 = 1
    V5P0 = 2
from enum import Enum

class RP2350:
    """
    A namespace class for the RP2350 microcontroller pin definitions.
    """
    class SPI0:
        class SCK(Enum):
            GPIO32 = 32
            GPIO34 = 34
            GPIO38 = 38
        
        class TX(Enum):
            GPIO35 = 35
            GPIO39 = 39
        
        class RX(Enum):
            GPIO36 = 36
            GPIO32 = 32

    class SPI1:
        class SCK(Enum):
            GPIO30 = 30
            GPIO42 = 42
            GPIO46 = 46

        class TX(Enum):
            GPIO31 = 31
            GPIO43 = 43
            GPIO47 = 47

        class RX(Enum):
            GPIO40 = 40
            GPIO44 = 44

        class CSn(Enum):
            GPIO41 = 41
            GPIO45 = 45

    class I2C0:
        class SDA(Enum):
            GPIO32 = 32
            GPIO36 = 36
            GPIO40 = 40
            GPIO44 = 44
        
        class SCL(Enum):
            GPIO33 = 33
            GPIO37 = 37
            GPIO41 = 41
            GPIO45 = 45
    
    class I2C1:
        class SDA(Enum):
            GPIO30 = 30
            GPIO34 = 34
            GPIO38 = 38
            GPIO42 = 42
            GPIO46 = 46

        class SCL(Enum):
            GPIO31 = 31
            GPIO35 = 35
            GPIO39 = 39
            GPIO43 = 43
            GPIO47 = 47

    class UART0:
        class TX(Enum):
            GPIO32 = 32
            GPIO44 = 44
        
        class RX(Enum):
            GPIO33 = 33
            GPIO45 = 45
        
        class CTS(Enum):
            GPIO30 = 30
            GPIO34 = 34
            GPIO46 = 46
        
        class RTS(Enum):
            GPIO31 = 31
            GPIO35 = 35
            GPIO47 = 47

    class UART1:
        class TX(Enum):
            GPIO36 = 36
            GPIO40 = 40
        
        class RX(Enum):
            GPIO37 = 37
            GPIO41 = 41
        
        class CTS(Enum):
            GPIO38 = 38
            GPIO42 = 42
        
        class RTS(Enum):
            GPIO39 = 39
            GPIO43 = 43

    class PWM7:
        class A(Enum):
            GPIO30 = 30
        
        class B(Enum):
            GPIO31 = 31

    class PWM8:
        class A(Enum):
            GPIO32 = 32
            GPIO40 = 40
        
        class B(Enum):
            GPIO33 = 33
            GPIO41 = 41
    
    class PWM9:
        class A(Enum):
            GPIO34 = 34
            GPIO42 = 42
        
        class B(Enum):
            GPIO35 = 35
            GPIO43 = 43
    
    class PWM10:
        class A(Enum):
            GPIO36 = 36
            GPIO44 = 44

        class B(Enum):
            GPIO37 = 37
            GPIO45 = 45
    
    class PWM11:
        class A(Enum):
            GPIO38 = 38
            GPIO46 = 46

        class B(Enum):
            GPIO39 = 39
            GPIO47 = 47