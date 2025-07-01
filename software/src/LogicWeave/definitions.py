import enum

class GPIOMode(enum.IntEnum):
    INPUT = 0
    OUTPUT = 1
    PWM = 2

class BankVoltage(enum.IntEnum):
    V1P8 = 0
    V3P3 = 1
    V5P0 = 2