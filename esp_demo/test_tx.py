from udp_conn import send_data

from network_events import generate_report
import time
import sys

if len(sys.argv) != 3:
    raise ValueError("Pass 2 arguments!")

ip_addr = sys.argv[1]
port = int(sys.argv[2])

send_data(ip_addr, port, generate_report(3, 4, 0))
time.sleep(2)
send_data(ip_addr, port, generate_report(3, 5, 1))
time.sleep(2)
send_data(ip_addr, port, generate_report(4, 2, 1))