from udp_conn import receive_data
from network_events import parse_report


receive_data("0.0.0.0", 8080, lambda x: print(parse_report(x)))
