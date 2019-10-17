# coding: utf-8
from robotUR import RobotUR

tcp_host_ip = '10.31.56.102'  # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = '10.31.56.102'  # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003

# on définit le robot
robot = RobotUR(tcp_host_ip, tcp_port, rtc_host_ip, rtc_port)
print("robot at home")

robot.joint_acc = 0.5
robot.joint_vel = 0.5
robot.tool_acc = 0.5

# on test la motricité du robot en déplacant le bras à une certaine position
robot.move_joints_degree([-270, -90, 0, -90 , 270, 180])

a = input("continuer?")

# on tente de fermer puis ouvrir la pince
robot.close_gripper()
robot.open_gripper()
