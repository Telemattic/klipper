# Support for Waveshare Servos

class PrinterSCUart:
    def __init__(self, config):
        ppins = config.get_printer().lookup_object("pins")
        uart_pin = config.get('uart_pin')
        pin_params = ppins.lookup_pin(uart_pin)
        self.printer = config.get_printer()
        self.mcu = pin_params['chip']
        self.pin = pin_params['pin']
        self.pio = config.get('pio', 0)
        self.baud = config.get('baud', 1_000_000)
        self.oid = self.mcu.create_oid()
        self.name = config.get_name().split()[1]
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SCUART_SEND", "SCUART", self.name,
                                   self.cmd_SCUART_SEND,
                                   desc=self.cmd_SCUART_SEND_help)
        self.mcu.register_config_callback(self.build_config)
    def build_config(self):
        self.mcu.add_config_cmd(
            "config_scuart oid=%d pio=%d pin=%s baud=%d"
            % (self.oid, self.pio, self.pin, self.baud))
        self.scuart_send_cmd = self.mcu.lookup_query_command(
            "scuart_send oid=%c write=%*s",
            "scuart_response oid=%c read=%*s", oid=self.oid,
            cq=self.cmd_queue, is_async=True)

    cmd_SCUART_SEND_help = "Send raw bytes (debugging)"
    def cmd_SCUART_SEND(self, gcmd):
        data = gcmd.get('DATA')
        self.scuart_send_cmd.send([self.oid, data])

def load_config_prefix(config):
    return PrinterSCUart(config)
