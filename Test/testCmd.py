import socket
import time

IP_UR5 = '10.31.56.104' # ATTENTION à bien changer l'adresse IP du robot

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(10)
s.connect((IP_UR5,30002))

tcp_command = 'movej([-4.712389,-1.570796,0.000000,-1.570796,4.712389,3.141593],a=0.500000,v=0.500000)\n'
s.send(str.encode(tcp_command))
time.sleep(10)  # On attend avoir d'envoyer la commande suivante que le movej se termine

tcp_command = "set_digital_out(8,True)\n"
s.send(str.encode(tcp_command)) # On tente d'ouvrir la pince (mais ça ne marche pas)
time.sleep(4)

tcp_command = "set_digital_out(8,False)\n"
s.send(str.encode(tcp_command))
time.sleep(4)

s.close()
