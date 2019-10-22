#from robotsim import RobotSim
from robot import Robot
import numpy as np
import time

workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])  # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)


#robot = RobotSim('objects/blocks', workspace_limits,True)

robot = Robot(True, 'objects/blocks', 10, workspace_limits,
              '10.31.56.102', 30002, '10.31.56.102', 30003,
              True, False, 'downloads/vpg-original-sim-pretrained-10-obj.pth')

robot.move_to([-0.7,0,0.1])

time.sleep(1)

robot.move_to([-0.5,0,0.2])


