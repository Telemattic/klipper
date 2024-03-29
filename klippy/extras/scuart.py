# Support for Waveshare Servos

class Inst:
    PING = 1
    READ = 2
    WRITE = 3

class Status:
    OK = 0
    TX_ERROR = 1
    RX_TIMEOUT = 2
    RX_DECODE = 3
    RX_CHECKSUM = 4
    UNKNOWN = 6

class PrinterSCUart:
    def __init__(self, config):
        ppins = config.get_printer().lookup_object("pins")
        pin_params = ppins.lookup_pin(config.get('pin'))
        self.printer = config.get_printer()
        self.mcu = pin_params['chip']
        self.pin = pin_params['pin']
        self.pio = config.get('pio', 0)
        self.baud = config.get('baud', 1_000_000)
        self.oid = self.mcu.create_oid()
        self.name = config.get_name().split()[1]
        self.cmd_queue = self.mcu.alloc_command_queue()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SCUART_RAWSEND", "SCUART", self.name,
                                   self.cmd_SCUART_RAWSEND,
                                   desc=self.cmd_SCUART_RAWSEND_help)
        gcode.register_mux_command("SCUART_PING", "SCUART", self.name,
                                   self.cmd_SCUART_PING,
                                   desc=self.cmd_SCUART_PING_help)
        gcode.register_mux_command("SCUART_READ", "SCUART", self.name,
                                   self.cmd_SCUART_READ,
                                   desc=self.cmd_SCUART_READ_help)
        gcode.register_mux_command("SCUART_WRITE", "SCUART", self.name,
                                   self.cmd_SCUART_WRITE,
                                   desc=self.cmd_SCUART_WRITE_help)
        self.mcu.register_config_callback(self.build_config)

    def build_config(self):
        self.mcu.add_config_cmd(
            "config_scuart oid=%d pio=%d pin=%s baud=%d"
            % (self.oid, self.pio, self.pin, self.baud))
        self.scuart_send_cmd = self.mcu.lookup_query_command(
            "scuart_send oid=%c write=%*s",
            "scuart_response oid=%c read=%*s", oid=self.oid,
            cq=self.cmd_queue, is_async=True)

    cmd_SCUART_RAWSEND_help = "Send raw bytes (debugging)"
    def cmd_SCUART_RAWSEND(self, gcmd):
        data = gcmd.get('DATA')
        params = self.scuart_send_cmd.send([self.oid, bytes.fromhex(data)])
        print(f"SCUART_RAWSEND: {params=}")

    cmd_SCUART_PING_help = "Ping"
    def cmd_SCUART_PING(self, gcmd):
        id = gcmd.get_int('ID')
        ret = self._send(id, Inst.PING, [])
        print(f"SCUART_PING: {id=} {ret=}")

    cmd_SCUART_READ_help = "Read"
    def cmd_SCUART_READ(self, gcmd):
        id = gcmd.get_int('ID')
        addr = gcmd.get_int('ADDR')
        rdlen = gcmd.get_int('LEN')
        ret = self._send(id, Inst.READ, [addr, rdlen])
        print(f"SCUART_READ: {id=} {addr=} len={rdlen} {ret=}")

    cmd_SCUART_WRITE_help = "Write"
    def cmd_SCUART_WRITE(self, gcmd):
        id = gcmd.get_int('ID')
        addr = gcmd.get_int('ADDR')
        data = bytes.fromhex(gcmd.get('DATA'))
        ret = self._send(id, Inst.WRITE, [addr] + list(data))
        print(f"SCUART_WRITE: {id=} {addr=} {data=} {ret=}")

    def _send(self, id, inst, data):
        msg = self._encode_send(id, inst, data)
        params = self.scuart_send_cmd.send([self.oid, msg])
        return self._decode_response(msg, params['read'])

    def _encode_send(self, id, inst, data):
        msg = [0xff, 0xff, id, 2 + len(data), inst] + data
        checksum = 0xff & ~sum(msg[2:])
        msg.append(checksum)
        return bytes(msg)

    def _decode_response(self, msg, response):
        if response[:len(msg)] != msg:
            return (Status.TX_ERROR, None)
        rx_data = response[len(msg):]
        if len(rx_data) == 0:
            return (Status.RX_TIMEOUT, None)
        for i, x in enumerate(rx_data):
            if x != 0xff:
                break
        if i < 2:
            return (Status.RX_DECODE, None)
        rx_data = rx_data[i:]
        if len(rx_data) < 3:
            return (Status.RX_DECODE, None)
        n = rx_data[1] + 2
        if len(rx_data) != n:
            return (Status.RX_DECODE, None)

        checksum = 0xff & ~sum(rx_data[:-1])
        if checksum != rx_data[-1]:
            return (Status.RX_CHECKSUM, None)

        # omit the length of the data, as it's implicit
        return (Status.OK, {'id':rx_data[0], 'status':rx_data[2], 'data':rx_data[3:-1]})

def load_config_prefix(config):
    return PrinterSCUart(config)
