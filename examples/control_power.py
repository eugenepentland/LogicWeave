from LogicWeave import LogicWeave, BankVoltage, GPIOMode
import time

with LogicWeave() as lw:
    ch1 = lw.pd_channel(1)

    ch2 = lw.pd_channel(2)

    ch1.request_power(voltage_mv=12000, current_limit_ma=1000)

    ch1.enable_output(True)

    print(ch1.read_status())