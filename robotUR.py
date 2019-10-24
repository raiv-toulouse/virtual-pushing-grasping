# coding: utf-8

import socket
import struct
import time
import numpy as np

NUMBER_BYTES_FROM_UR = 1116  # For the UR3.10 version
INTEGER = 1
DOUBLE = 2
ARRAY_OF_6_DOUBLES = 3


class RobotUR(object):
    def __init__(self, tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,joint_acc=1.4,joint_vel=1.05):

        # Default home joint configuration
        # self.home_joint_config = [-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        self.home_joint_config = [-(180.0 / 360.0) * 2 * np.pi, -(84.2 / 360.0) * 2 * np.pi,
                                  (112.8 / 360.0) * 2 * np.pi, -(119.7 / 360.0) * 2 * np.pi,
                                  -(90.0 / 360.0) * 2 * np.pi, 0.0]

        # Default joint speed configuration
        self.joint_acc = joint_acc # Safe: 1.4
        self.joint_vel = joint_vel  # Safe: 1.05

        # Joint tolerance for blocking calls
        self.joint_tolerance = 0.01

        # Network parameters to connect to robot client
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port

        # Network parameters to connect as real-time client to parse state data
        self.rtc_host_ip = rtc_host_ip
        self.rtc_port = rtc_port

        # The state dictionnary to collect all the info from UR robot
        self.dicoVariables = {'actualCartesianCoordinatesOfTool': (444, ARRAY_OF_6_DOUBLES),
                              'actualJointPositions': (252, ARRAY_OF_6_DOUBLES)}

        # Move robot to home pose
        #self.go_home()

    def go_home(self):
        self.move_joints(self.home_joint_config)

    def move_to(self, tool_position, tool_orientation):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # Build the movel command
        tcp_command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (
            tool_position[0], tool_position[1], tool_position[2], tool_orientation[0], tool_orientation[1],
            tool_orientation[2], self.tool_acc, self.tool_vel)
        self.tcp_socket.send(str.encode(tcp_command))
        # Block until robot reaches target tool position
        dicoState = self.get_state()
        actual_tool_pose = dicoState['actualCartesianCoordinatesOfTool']
        while not all([np.abs(actual_tool_pose[j] - tool_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            dicoState = self.get_state()
            actual_tool_pose = dicoState['actualCartesianCoordinatesOfTool']
        self.tcp_socket.close()

    def move_joints(self, joint_configuration):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # Build the movej command
        tcp_command = "movej([%f" % joint_configuration[0]
        for joint_idx in range(1, 6):
            tcp_command = tcp_command + (",%f" % joint_configuration[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f)\n" % (self.joint_acc, self.joint_vel)
        print(tcp_command)
        self.tcp_socket.send(str.encode(tcp_command))
        # # Block until robot reaches home state
        dicoState = self.get_state()
        actual_joint_positions = dicoState['actualJointPositions']
        while not all([np.abs(actual_joint_positions[j] - joint_configuration[j]) < self.joint_tolerance for j in range(6)]):
            dicoState = self.get_state()
            actual_joint_positions = dicoState['actualJointPositions']
            print(actual_joint_positions)
        self.tcp_socket.close()

    # Avec les 6 angles exprimés en degré
    def move_joints_degree(self, joint_configuration):
        self.move_joints([q*np.pi/180 for q in joint_configuration])

    def open_gripper(self, async=False):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "RG2(50)\n"
        #tcp_command = "set_digital_out(8,False)\n"
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()
        if not async:
            time.sleep(1.5)

    def close_gripper(self, async=False):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "RG2(10)\n"
        #tcp_command = "set_digital_out(8,True)\n"
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()
        if not async:
            time.sleep(1.5)

    def isGripperFullyClosed(self, async=False):
        if async:
            gripper_fully_closed = True
        else:
            time.sleep(1.5)
            gripper_fully_closed = self.check_grasp()
        return gripper_fully_closed

    def check_grasp(self):
        state_data = self.get_state()
        tool_analog_input2 = self.parse_tcp_state_data(state_data, 'tool_data')
        return tool_analog_input2 > 0.26

    # Get all the information from robot UR through RealTime port 30003
    # See Excel file for the list of available data
    def get_state(self):
        # Connect as real-time client to parse state data
        rtc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rtc_socket.connect((self.rtc_host_ip, self.rtc_port))
        stateData = rtc_socket.recv(NUMBER_BYTES_FROM_UR)
        dico = {}
        for key, (offset, dataType) in self.dicoVariables.items():
            if dataType == INTEGER:
                dico[key] = self.getInteger(stateData, offset)
            if dataType == DOUBLE:
                dico[key] = self.getDouble(stateData, offset)
            if dataType == ARRAY_OF_6_DOUBLES:
                dico[key] = self.getListOf6Double(stateData, offset)
        rtc_socket.close()
        return dico

    def getDouble(self, data, pos):
        return struct.unpack('!d', data[pos:pos + 8])[0]  # d car double

    def getInteger(self, data, pos):
        return struct.unpack('!i', data[pos:pos + 4])[0]  # i car integer

    def getListOf6Double(self, data, pos):
        return struct.unpack('!dddddd', data[pos:pos + 48])  # d car double et 48 car 6 doubles * 8 octets

#
# Programme principal
#
if __name__ == '__main__':
    IP_UR3 = '10.31.56.102'
    IP_UR5 = '10.31.56.104'
    monRobot = RobotUR(IP_UR5, 30002, IP_UR5, 30003,0.5,0.5)
    print('premier mouvement')
    monRobot.move_joints([-3.141593,-1.469567,1.968731,-2.089159,-1.570796,0.000000])
    print()
    print('deuxième mouvement')
    print()
    monRobot.move_joints([-3.141593,-1.570796,1.570796,0.000000,1.570796,3.141593])
    print()
    monRobot.open_gripper()
    time.sleep(5)
    monRobot.close_gripper()
