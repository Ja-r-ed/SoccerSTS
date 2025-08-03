import socket
import struct
import time

TCP_IP = '192.168.1.66'
TCP_PORT = 10000
BUFFER_SIZE = 1024
value = 3.145

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
while True:
    ba = bytearray(struct.pack("f", value)) 
    s.send(ba)
    value=value+5.0
    time.sleep(3)
s.close()