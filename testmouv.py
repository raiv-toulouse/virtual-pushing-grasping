from robotUR import RobotUR
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import time

tcp_host_ip = '10.31.56.102'  # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = '10.31.56.102'  # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003
workspace_limits = np.asarray([[0.3, 0.748], [0.05, 0.4], [-0.2,
                                                           -0.1]])  # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
calib_grid_step = 0.05

# on définit le robot
robot = RobotUR(None, None, workspace_limits,
                tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
                False, None, None)

robot.joint_acc = 0.5
robot.joint_vel = 0.5
robot.tool_acc = 0.5


# on test la motricité du robot en déplacant le bras à une certaine position
# Make robot gripper point upwards
robot.move_joints([-np.pi, -np.pi / 2, np.pi / 2, 0 , np.pi / 2, np.pi])

a = input("continuer?")

# on tente de fermer ou ouvrir la pince

robot.close_gripper()
