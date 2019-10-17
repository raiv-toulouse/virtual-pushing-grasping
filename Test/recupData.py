# Echo client program
import socket
import time
import struct


HOST = "10.31.56.102"  # The remote host
PORT = 30003           #  RealTime port

def getDoubleFromSocket(s):
    packet = s.recv(8)
    return struct.unpack('!d', packet)[0]  # d car double

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
        packetSize = s.recv(4)
        totalSize = struct.unpack('!I', packetSize)[0]  # I car integer
        print("Total size = ", totalSize)
        offset = 8+9*48
        _ = s.recv(offset)  # Unused data
        # We read 6 * 8 bytes (size of a double)
        print("X = ", getDoubleFromSocket(s) * 1000)
        print("Y = ", getDoubleFromSocket(s) * 1000)
        print("Z = ", getDoubleFromSocket(s) * 1000)
        print("Rx = ", getDoubleFromSocket(s))
        print("Ry = ", getDoubleFromSocket(s))
        print("Rz = ", getDoubleFromSocket(s))

        _ = s.recv(totalSize - 4 - offset - 6 * 8)  # To read all the remaining data on this packet
except socket.error as socketerror:
    print("Error: ", socketerror)

