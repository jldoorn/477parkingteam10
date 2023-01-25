from wifi_setup import *
import sys
from network_events import *
if len(sys.argv) != 2:
    raise ValueError("Not enough args")

port = serial.Serial(sys.argv[1], 115200)
test_comport(port)
set_soft_ap(port)
set_wifi_link(port, "myapteam10", "012345678")
listen_for_udp(port, 8080)

while True:

    if port.out_waiting > 0:
        out = read_esp(port)
        parsed = parse_esp_output(out)

        for p in parsed:
            if type(p) == ESPPacket:
                print(parse_report(p.data_in))
            else:
                print(f"Got a status from ESP: {p.line}")

