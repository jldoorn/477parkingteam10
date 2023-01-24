
import serial
import os
import sys

def parse_at_output(_in: bytes):
    pass

def write_flush_getline(port: serial.Serial, to_write: bytes) -> bytes :
    port.write(to_write)
    # port.flush()
    # port.readline()
    # port.flush()
    # return port.readline()
    return port.read_until(b'OK\r\n')

def test_comport(port: serial.Serial):
#    assert write_flush_getline(port, b'AT\r\n') ==  b'OK\r\n'
    print(write_flush_getline(port, b'AT\r\n'))

def set_soft_ap(port: serial.Serial):
    write_flush_getline(port, b'AT+CWMODE=2\r\n')

def set_wifi_link(port, ssid, password):
    write_flush_getline(port, b'AT+CWSAP="' + bytes(ssid, "ascii") + b'","' + bytes(password, "ascii") + b'",5,3\r\n')

if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise ValueError("Not enough args")
    comport = sys.argv[1]
    
    port = serial.Serial(comport, 115200)
    test_comport(port)
    set_soft_ap(port)
    set_wifi_link(port, "myap", "012345678")
