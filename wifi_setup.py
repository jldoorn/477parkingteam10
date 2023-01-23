
import serial
import os
import sys

def parse_at_output(_in: bytes):
    pass

def write_flush_getline(port: serial.Serial, to_write: bytes) -> bytes :
    port.write(to_write)
    port.flush()
    return port.readline()

def test_comport(port: serial.Serial):
   assert write_flush_getline(port, b'AT\r\n') ==  b'OK\r\n'

def set_soft_ap(port: serial.Serial):
    write_flush_getline(port, b'AT+CWMODE=2\r\n')


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise ValueError("Not enough args")
    comport = sys.argv[1]
    
    port = serial.Serial(comport, 115200)
    test_comport(port)