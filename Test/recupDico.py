# Echo client program
import socket
import time
import struct


HOST = "10.31.56.102"  # The remote host
PORT = 30003           #  RealTime port
NUMBER_BYTES_FROM_UR = 1116  # For the UR3.10 version

def getDouble(data,pos):
    return struct.unpack('!d', data[pos:pos+8])[0]  # d car double

#
# Main program
#
print("Starting Program")

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10)
    s.connect((HOST, PORT))

    while True:
        time.sleep(1.00)
        print()
        data = s.recv(NUMBER_BYTES_FROM_UR)
        totalSize = struct.unpack('!I', data[:4])[0]  # I car integer
        print("Total size = ", totalSize)
        offset = 4+8+9*48
        # We read 6 * 8 bytes (size of a double)
        print("X = ", getDouble(data,offset) * 1000)
        print("Y = ", getDouble(data,offset+8) * 1000)
        print("Z = ", getDouble(data,offset+2*8) * 1000)
        print("Rx = ", getDouble(data,offset+3*8))
        print("Ry = ", getDouble(data,offset+4*8))
        print("Rz = ", getDouble(data,offset+5*8))
        print()
        offset = 252
        print("q1 = ", getDouble(data, offset))
        print("q2 = ", getDouble(data, offset + 8))
        print("q3 = ", getDouble(data, offset + 2 * 8))
        print("q4 = ", getDouble(data, offset + 3 * 8))
        print("q5 = ", getDouble(data, offset + 4 * 8))
        print("q6 = ", getDouble(data, offset + 5 * 8))
except socket.error as socketerror:
    print("Error: ", socketerror)

