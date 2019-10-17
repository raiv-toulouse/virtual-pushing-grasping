import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(10)
s.connect(('10.31.56.104',30002))
time.sleep(2)
tcp_command = 'movej([-4.712389,-1.570796,0.000000,-1.570796,4.712389,3.141593],a=0.500000,v=0.500000)\n'
s.send(str.encode(tcp_command))
time.sleep(10)
tcp_command = "set_digital_out(8,True)\n"
s.send(str.encode(tcp_command))
time.sleep(4)
tcp_command = "set_digital_out(8,False)\n"
s.send(str.encode(tcp_command))
time.sleep(4)
s.close()
