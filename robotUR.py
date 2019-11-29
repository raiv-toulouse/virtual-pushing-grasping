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
        self.tcp_socket.close()

    # Avec les 6 angles exprimés en degré
    def move_joints_degree(self, joint_configuration):
        self.move_joints([q*np.pi/180 for q in joint_configuration])

    def open_gripper(self, async=False):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "set_digital_out(8,False)\n"
        #tcp_command = "set_digital_out(8,False)\n"
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()
        if not async:
            time.sleep(1.5)

    def close_gripper(self, async=False):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "set_digital_out(8,True)\n"
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
        tool_analog_input2 = self.parse_tcp_state_data( 'tool_data')
        return tool_analog_input2 > 0.26

    # Get all the information from robot UR through RealTime port 30003
    # See Excel file for the list of available data
    def get_state(self):
        # Connect as real-time client to parse state data
        rtc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rtc_socket.connect((self.rtc_host_ip, self.
                            rtc_port))
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

    def readMessage(self,s):
        msgOK = False
        while msgOK != True:
            header = s.recv(4)
            # Read package header
            data_bytes = bytearray()
            data_bytes.extend(header)
            data_length = struct.unpack("!i", data_bytes[0:4])[0]
            # Read message
            message = s.recv(data_length - 4)  # 4 is for the previously read header
            data_bytes.extend(message)
            if message[0] == 16:
                msgOK = True
        return message, data_length

    def parse_tcp_state_data(self,subpackage):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.tcp_host_ip, self.tcp_port))
        data_bytes,data_length = self.readMessage(s)
        s.close()
        robot_message_type = data_bytes[0]
        assert (robot_message_type == 16)
        byte_idx = 1
        # Parse sub-packages
        subpackage_types = {'joint_data': 1, 'cartesian_info': 4, 'force_mode_data': 7, 'tool_data': 2}
        while byte_idx < data_length:
            # package_length = int.from_bytes(data_bytes[byte_idx:(byte_idx+4)], byteorder='big', signed=False)
            package_length = struct.unpack("!i", data_bytes[byte_idx:(byte_idx + 4)])[0]
            byte_idx += 4
            package_idx = data_bytes[byte_idx]
            if package_idx == subpackage_types[subpackage]:
                byte_idx += 1
                break
            byte_idx += package_length - 4

        def parse_joint_data(data_bytes, byte_idx):
            actual_joint_positions = [0, 0, 0, 0, 0, 0]
            target_joint_positions = [0, 0, 0, 0, 0, 0]
            for joint_idx in range(6):
                actual_joint_positions[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx + 0):(byte_idx + 8)])[0]
                target_joint_positions[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx + 8):(byte_idx + 16)])[0]
                byte_idx += 41
            return actual_joint_positions

        def parse_cartesian_info(data_bytes, byte_idx):
            actual_tool_pose = [0, 0, 0, 0, 0, 0]
            for pose_value_idx in range(6):
                actual_tool_pose[pose_value_idx] = struct.unpack('!d', data_bytes[(byte_idx + 0):(byte_idx + 8)])[0]
                byte_idx += 8
            return actual_tool_pose

        def parse_tool_data(data_bytes, byte_idx):
            byte_idx += 2
            tool_analog_input2 = struct.unpack('!d', data_bytes[(byte_idx + 0):(byte_idx + 8)])[0]
            return tool_analog_input2

        parse_functions = {'joint_data': parse_joint_data, 'cartesian_info': parse_cartesian_info,
                           'tool_data': parse_tool_data}
        return parse_functions[subpackage](data_bytes, byte_idx)

#
# Programme principal
#
if __name__ == '__main__':
    IP_UR3 = '10.31.56.102'  # IP and port to robot arm as TCP client (UR5)
    IP_UR5 = '10.31.56.104'
    tcp_port = 30002
    rtc_port = 30003
    print("robot at home")
    monRobot = RobotUR(IP_UR3, tcp_port, IP_UR3, rtc_port,0.5,0.5)
    print(monRobot.parse_tcp_state_data('tool_data'))
    print(monRobot.check_grasp())
    print('on ouvre la pince')
    print(monRobot.check_grasp())
    monRobot.open_gripper()
    time.sleep(5)
    print(monRobot.check_grasp())
    print('on ferme la pince')
    monRobot.close_gripper()
    print(monRobot.check_grasp())
    print('fin du premier mouvement')
    monRobot.move_joints([-3.141593,-1.469567,1.968731,-2.089159,-1.570796,0.000000])
    print('fin du deuxième mouvement')
    monRobot.move_joints([-3.141593,-1.570796,1.570796,0.000000,1.570796,3.141593])
    print("déplacement en degré")
    monRobot.move_joints_degree([-270, -90, 0, -90, 270, 180])
    print('fin du programme de test')
