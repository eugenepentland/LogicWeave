from LogicWeave import LogicWeave, BankVoltage, GPIOMode

with LogicWeave() as lw:
    # Print the firmware version
    lw.write_bank_voltage(1, BankVoltage.V1P8)
    lw.write_bank_voltage(2, BankVoltage.V3P3)
    lw.write_bank_voltage(3, BankVoltage.V5P0)

    for pin in [30,37,41]:
        lw.write_gpio_mode(pin, GPIOMode.OUTPUT)
        lw.gpio_write(pin, True)