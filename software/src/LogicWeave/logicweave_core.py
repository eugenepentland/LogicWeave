
from LogicWeave import LogicWeave
import LogicWeave.proto_gen.logicweave_core_pb2 as logicweave_core_pb2

class LogicWeaveCore(LogicWeave):
    def __init__(self, *args, **kwargs):
        kwargs['protobuf_module'] = logicweave_core_pb2
        super().__init__(*args, **kwargs)

    def read_voltage(self):
        request = self.pb.ReadVoltageRequest()
        return self._send_and_parse(request, "read_voltage_response")

    def read_resistance(self):
        request = self.pb.ReadResistanceRequest()
        return self._send_and_parse(request, "read_resistance_response")
    
    def read_pd(self):
        request = self.pb.ReadPDRequest()
        return self._send_and_parse(request, "read_pd_response")
    
    def set_psu_output(self, channel, state):
        request = self.pb.SetPSUOutputRequest(channel=channel, state=state)
        return self._send_and_parse(request, "set_psu_output_response")
    
    def read_power_monitor(self):
        request = self.pb.ReadPowerMonitorRequest()
        return self._send_and_parse(request, "read_power_monitor_response")
    
    def configure_psu(self, channel, voltage, current_limit):
        request = self.pb.ConfigurePSURequest(channel=channel, voltage=voltage, current_limit=current_limit)
        return self._send_and_parse(request, "configure_psu_response")
    
    def cal_probes(self):
        request = self.pb.ZeroProbesRequest()
        return self._send_and_parse(request, "zero_probes_response")

    def read_calibration_data(self):
        request = self.pb.ReadCalibrationDataRequest()
        return self._send_and_parse(request, "read_calibration_data_response")