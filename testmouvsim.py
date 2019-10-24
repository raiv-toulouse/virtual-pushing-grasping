from robotsim import RobotSim
import numpy as np
import time

workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])  # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)


robot = RobotSim('objects/blocks', workspace_limits,
                True)




robot.move_to([-0.5,0,0])

robot.close_gripper()

robot.move_to([-0.5,0,0.5])












