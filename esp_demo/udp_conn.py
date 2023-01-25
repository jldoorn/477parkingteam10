import socket
import sys


def receive_data(ip_addr: str, port: int, handler_func=None):

    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind((ip_addr, port))

    while True:
        (data, addr) = sock.recvfrom(1024)
        
        if handler_func is None:
            print(f"rx message from {addr}: {data}")
        else:
            handler_func(data)
        

def send_data(ip_addr: str, port: int, data: bytes):

    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    sock.sendto(data, (ip_addr, port))


if __name__ == "__main__":
    if sys.argv[1] == "rx":
        receive_data("0.0.0.0", 8080)
    else:
        send_data("0.0.0.0", 8080, sys.argv[1].encode("ascii"))
