
import serial
import os
import sys
import typing
import time

class ESPEvent:

    def __init__(self, line: bytes, **kwargs) -> None:
        self.line = line
        

class ESPPacket(ESPEvent):

    def __init__(self, line, data_in, **kwargs) -> None:
        super().__init__(line)
        self.data_in = data_in

def read_esp(port: serial.Serial) -> typing.List[bytes]:
    out = []
    while port.in_waiting > 0:
        port.flush()
        l = port.read(port.in_waiting).split(b"\r\n")
        out.extend([b for b in l if b != b''])

    return out

def handle_esp_plus(line: bytes):
    return ESPEvent(line)

# esp_out = [b'+IPD,0,23:asfd;jhh slfhjads fjk a\r\n',b'+IPD,0,2:\r\n',b'\r\n']
def parse_esp_output(output: typing.List[bytes]):
    if len(output) == 0:
        raise ValueError("Output must have a value in list.")
    parsed = []
    for line in output:
        if line.startswith(b'+IPD'): # incomming data!
            print("incomming data", line)
            msg_parts = line.split(b':')
            msg_len = int(msg_parts[0].split(b',')[-1])

            data_in = msg_parts[1][:msg_len]
            parsed.append(ESPPacket(line, data_in))
            continue

        if line.startswith(b'AT'): # an echo
            continue

        if line.startswith(b'+'):
            parsed.append(handle_esp_plus(line))
            continue

        if line.startswith(b'ERROR'):
            print("got an error")
            continue

        if line.startswith(b'\r\n'):
            continue

    return parsed


def write_flush_getline(port: serial.Serial, to_write: bytes) -> bytes :
    port.write(to_write)
    # port.flush()
    # port.readline()
    # port.flush()
    # return port.readline()
    return port.read_until(b'OK\r\n')

def test_comport(port: serial.Serial):
#    assert write_flush_getline(port, b'AT\r\n') ==  b'OK\r\n'
    port.write(b'AT\r\n')
    time.sleep(0.1)
    out = read_esp(port)
    assert out[-1].startswith(b'OK')

def set_soft_ap(port: serial.Serial):
    port.write(b'AT+CWMODE=2\r\n')
    time.sleep(0.1)
    out = read_esp(port)
    assert out[-1].startswith(b'OK')

def set_wifi_link(port, ssid, password):
    port.write(b'AT+CWSAP="' + bytes(ssid, "ascii") + b'","' + bytes(password, "ascii") + b'",5,3\r\n')
    time.sleep(0.5)
    out = read_esp(port)
    assert out[-1].startswith(b'OK')

def listen_for_udp(port, udpport):
    port.write(b'AT+CIPMUX=1\r\n')
    time.sleep(0.1)
    out = read_esp(port)
    assert out[-1].startswith(b'OK')

    port.write(b'AT+CIPSTART=0,"UDP","0.0.0.0",' + str(udpport).encode("ascii") + b','+str(udpport).encode("ascii") + b',2\r\n')
    time.sleep(0.1)
    out = read_esp(port)
    assert out[-1].startswith(b'OK')

def send_to_udp(ser_port, ip_addr, port, data_out):
    ser_port.write(b'AT+CIPSEND=' + str(len(data_out)).encode("ascii") + b',"' + ip_addr.encode("ascii") + b'",' + str(port).encode("ascii") + b'\r\n')
    time.sleep(0.1)
    out = read_esp(port)
    assert out[-1].startswith(b'>')

    ser_port.write(data_out)
    time.sleep(0.1)
    out = read_esp(ser_port)
    assert out[-1].startswith(b'SEND OK')

def join_ap(port, ssid, password):
    port.write(b'AT+CWJAP="' + ssid.encode("ascii") + b'","' + password.encode("ascii"), + b'"\r\n')
    out = read_esp(port)
    assert out[-1].startswith(b'OK')

if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise ValueError("Not enough args")
    comport = sys.argv[1]
    
    port = serial.Serial(comport, 115200)
    test_comport(port)
    set_soft_ap(port)
    set_wifi_link(port, "myap", "012345678")
    listen_for_udp(port, 8080)
    while True:
        print(port.readline())
