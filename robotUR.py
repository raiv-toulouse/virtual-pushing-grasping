import socket
import select
import struct
import time
import os
import numpy as np
import utils
from simulation import vrep


class RobotUR(object):
    def __init__(self, obj_mesh_dir, num_obj, workspace_limits,
                 tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
                 is_testing, test_preset_cases, test_preset_file):
        self.workspace_limits = workspace_limits

        # Connect to robot client
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port
        # self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Connect as real-time client to parse state data
        self.rtc_host_ip = rtc_host_ip
        self.rtc_port = rtc_port

        # Default home joint configuration
        # self.home_joint_config = [-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        self.home_joint_config = [-(180.0 / 360.0) * 2 * np.pi, -(84.2 / 360.0) * 2 * np.pi,
                                  (112.8 / 360.0) * 2 * np.pi, -(119.7 / 360.0) * 2 * np.pi,
                                  -(90.0 / 360.0) * 2 * np.pi, 0.0]

        # Default joint speed configuration
        self.joint_acc = 1.4  # Safe: 1.4
        self.joint_vel = 1  # Safe: 1.05

        # Joint tolerance for blocking calls
        self.joint_tolerance = 0.01

        # Move robot to home pose
        self.go_home()



        self.tcp_socket.close()


    def move_to(self, tool_position, tool_orientation):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (
            tool_position[0], tool_position[1], tool_position[2], tool_orientation[0], tool_orientation[1],
            tool_orientation[2], self.tool_acc, self.tool_vel)
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches target tool position
        tcp_state_data = self.tcp_socket.recv(2048)
        actual_tool_pose = self.parse_tcp_state_data(tcp_state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_pose[j] - tool_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            # [min(np.abs(actual_tool_pose[j] - tool_orientation[j-3]), np.abs(np.abs(actual_tool_pose[j] - tool_orientation[j-3]) - np.pi*2)) < self.tool_pose_tolerance[j] for j in range(3,6)]
            # print([np.abs(actual_tool_pose[j] - tool_position[j]) for j in range(3)] + [min(np.abs(actual_tool_pose[j] - tool_orientation[j-3]), np.abs(np.abs(actual_tool_pose[j] - tool_orientation[j-3]) - np.pi*2)) for j in range(3,6)])
            tcp_state_data = self.tcp_socket.recv(2048)
            prev_actual_tool_pose = np.asarray(actual_tool_pose).copy()
            actual_tool_pose = self.parse_tcp_state_data(tcp_state_data, 'cartesian_info')
            time.sleep(0.01)

        self.tcp_socket.close()


    def move_joints(self, joint_configuration):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "movej([%f" % joint_configuration[0]
        for joint_idx in range(1, 6):
            tcp_command = tcp_command + (",%f" % joint_configuration[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f)\n" % (self.joint_acc, self.joint_vel)
        print(tcp_command)
        self.tcp_socket.send(str.encode(tcp_command))
        #
        # # Block until robot reaches home state
        # state_data = self.tcp_socket.recv(2048)
        # actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
        # while not all(
        #         [np.abs(actual_joint_positions[j] - joint_configuration[j]) < self.joint_tolerance for j in range(6)]):
        #     state_data = self.tcp_socket.recv(2048)
        #     actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
        #     time.sleep(0.01)
        #
        self.tcp_socket.close()

    def go_home(self):
        self.move_joints(self.home_joint_config)

    def parse_tcp_state_data(self, state_data, subpackage):
        print('parse_tcp_state_data')
        print(state_data)
        # Read package header
        data_bytes = bytearray()
        data_bytes.extend(state_data)
        data_length = struct.unpack("!i", data_bytes[0:4])[0];
        print(data_length)
        print(data_bytes)
        robot_message_type = data_bytes[4]
        # assert(robot_message_type == 16)
        byte_idx = 5

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
            print("byte_idx = ", byte_idx)

        def parse_joint_data(data_bytes, byte_idx):
            actual_joint_positions = [0, 0, 0, 0, 0, 0]
            target_joint_positions = [0, 0, 0, 0, 0, 0]
            for joint_idx in range(6):
                actual_joint_positions[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx + 0):(byte_idx + 8)])[0]
                target_joint_positions[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx + 8):(byte_idx + 16)])[0]
                print("Coord n° {} = {}".format(joint_idx, actual_joint_positions[joint_idx]))
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



    def close_gripper(self, async=False):

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "set_digital_out(8,True)\n"
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()
        if not async:
            time.sleep(1.5)



    def isGripperFullyClosed(self,async=False):
        if async:
            gripper_fully_closed = True
        else:
            time.sleep(1.5)
            gripper_fully_closed = self.check_grasp()
        return gripper_fully_closed


    def open_gripper(self, async=False):

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "set_digital_out(8,False)\n"
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()
        if not async:
            time.sleep(1.5)

    def check_grasp(self):

        state_data = self.get_state()
        tool_analog_input2 = self.parse_tcp_state_data(state_data, 'tool_data')
        return tool_analog_input2 > 0.26

    def get_state(self):

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        state_data = self.tcp_socket.recv(2048)
        self.tcp_socket.close()
        return state_data








