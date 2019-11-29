import socket
import struct

HOST = "10.31.56.102"  # The remote host
PORT = 30002  # The same port as used by the server

def readMessage(s):
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


def parse_tcp_state_data(subpackage):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    data_bytes,data_length = readMessage(s)
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


for i in range(7):
    print(parse_tcp_state_data('tool_data'))
