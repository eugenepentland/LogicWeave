import LogicWeave

with LogicWeave() as lw:
    # Print the firmware version
    print(lw.read_firmware_info())

    # Set the voltage/current limits
    lw.write_pd_power_request(channel=1, voltage_mv=9000, current_limit_ma=1250)

    # Turn the channel on/off
    lw.write_pd_output_state(channel=1, on=True)

    # Read what its currently set to
    print(lw.read_pd_channel_status(1))