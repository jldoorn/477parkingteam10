import struct

class CarFlowEvent:

    def __init__(self, serial_number, seq_num, direction) -> None:
        self.serial_number = serial_number
        self.seq_num = seq_num
        self.direction = direction
    
    def __str__(self) -> str:
        direction = "out of" if self.direction == 0 else "into"

        return f"[CDM{self.serial_number}:SEQ{self.seq_num}] Car has gone {direction} the garage"

def parse_report(data_in: bytes):

    header = data_in[:2]
    serial_number, api_code = struct.unpack('>BB', header)
    if api_code == 2:
        # log event
        seq_num, direction = struct.unpack('>BB', data_in[2:4])
        return CarFlowEvent(serial_number, seq_num, direction)

def generate_report(serial, sequence, direction):
    return struct.pack('>BBBB', serial, 2, sequence, direction)

if __name__ == "__main__":
    print(parse_report(struct.pack('>BBBB', 12, 2, 4, 1)))
    print(parse_report(struct.pack('>BBBB', 10, 2, 199, 0)))
