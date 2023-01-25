from wifi_setup import *
import sys
from network_events import *
if len(sys.argv) != 2:
    raise ValueError("Not enough args")

print(sys.argv[1])
port = serial.Serial(sys.argv[1], 115200)
print("testing port")
test_comport(port)
print("setting AP")
set_soft_ap(port)
print("setting link")
set_wifi_link(port, "myapteam10", "012345678")
print("listening for  udp")
listen_for_udp(port, 8080)

print("entering loop")
while True:
    time.sleep(0.2)
    if port.in_waiting > 0:
        print("heard something")
        out = read_esp(port)
        parsed = parse_esp_output(out)
        for p in parsed:
            if type(p) == ESPPacket:
                print(parse_report(p.data_in))
            else:
                print(f"Got a status from ESP: {p.line}")

