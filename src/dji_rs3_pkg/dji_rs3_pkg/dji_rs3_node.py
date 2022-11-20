#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from wildpose_interfaces.msg import DjiRsStatus

import can
import struct
import numpy as np
import time
import ctypes
from typing import List, Any, Dict
from .protocol.sdk import CmdCombine
from .protocol.sdk.SDKCRC import calc_crc


BITRATE = 1000000
CAN_LENQ = 8
Seq_Init_Data = 0x0002


def bytearray2string(data):
    return ':'.join(['{:02X}'.format(d) for d in data])


def _check_head_crc(data):
    cmd_prefix = bytearray2string(data[:-2])
    crc16 = calc_crc(cmd_prefix, 16)
    recv_crc = bytearray2string(data[-2:])

    return crc16 == recv_crc

def _check_pack_crc(data):
    cmd_prefix = bytearray2string(data[:-4])
    crc16 = calc_crc(cmd_prefix, 32)
    recv_crc = bytearray2string(data[-4:])

    return crc16 == recv_crc


def bytearray2value(bytes, signed=False) -> int:
    if len(bytes) > 1:
        value = (bytes[1] << 8) + bytes[0]
    else:
        value = bytes[0]

    if signed:
        return ctypes.c_short(value).value
    else:
        return value


def command_generator(cmd_type: str, cmd_set: str, cmd_id: str, data: str) -> bytearray:
    dji_cmd = CmdCombine.combine(cmd_type=cmd_type, cmd_set=cmd_set, cmd_id=cmd_id, data=data)
    return [int(b, 16) for b in dji_cmd.split(':')]


def joymsg2f510(msg):
    # Logitech F510
    # Button: [A, B, X, Y, LB, RB, BACK, START, Logitech, Right joy, Left joy]
    # Axes: [JoyL LR(+-), JoyL UB(+-), LT(-1~1), JoyR LR(+-), JoyR UB(+-), RT(-1~1), D-pad LR(+-), D-pad UB(+-)]
    buttons = {
        'A': msg.buttons[0],
        'B': msg.buttons[1],
        'X': msg.buttons[2],
        'Y': msg.buttons[3],
        'LB': msg.buttons[4],
        'RB': msg.buttons[5],
        'BACK': msg.buttons[6],
        'START': msg.buttons[7],
        'Logitech': msg.buttons[8],
        'joy_left': msg.buttons[9],
        'joy_right': msg.buttons[10],
    }
    axes = {
        'joy_left': [msg.axes[0], msg.axes[1]],
        'LT': msg.axes[2],
        'joy_right': [msg.axes[3], msg.axes[4]],
        'RT': msg.axes[5],
        'd_pad': [msg.axes[6], msg.axes[7]],
    }
    
    return buttons, axes


class DjiRs3Node(Node):
    def __init__(self):
        super().__init__("dji_rs3_node")

        # Node parameters
        self.declare_parameter("channel", "can0")
        self.channel_ = self.get_parameter("channel").value
        
        self.joy_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.rs3_status_publisher_ = self.create_publisher(DjiRsStatus, 'dji_rs3', 10)

        self.bus_ = can.interface.Bus(
            bustype='socketcan',
            channel=self.channel_,
            bitrate=BITRATE,
        )
        self.send_id_ = 0x223
        self.rev_id_ = 0x222
        self.timeout_ = 0.1
        
        self.cmd_list_ = []

        self.recenter()
        # self.move_to(dyaw=90, dpitch=0, droll=0, time_ms=0)
        self.timer_ = self.create_timer(0.1, self.loop)

        self.get_logger().info("dji_rs3_node started.")
        
    def joy_callback(self, msg):
        buttons, axes = joymsg2f510(msg)
        if buttons['joy_right'] == 1:
            # move to the home position
            self.recenter()
        elif axes['joy_right'][0] != 0:
            rl = - axes['joy_right'][0]
            self.move_to(dyaw=rl * 10)
        elif axes['joy_right'][1] != 0:
            ub = axes['joy_right'][1]
            self.move_to(dpitch=ub * 10)
            
    def send_can_message(self, cmd: List):
        for i in range(0, len(cmd), CAN_LENQ):
            data = bytearray(cmd[i:i+CAN_LENQ])
            msg = can.Message(
                arbitration_id=self.send_id_,
                is_extended_id=False,
                is_rx=False,
                data=data
            )
            try:
                self.bus_.send(msg, timeout=self.timeout_)
                self.get_logger().debug(f'Message sent {bytearray2string(data)} on {self.bus_.channel_info}')
            except can.CanError:
                self.get_logger().error("Faild to send a CAN message.")
            
    def recenter(self,):
        hex_data = struct.pack(
            '<2B',    # format: https://docs.python.org/3/library/struct.html#format-strings
            0xFE,   # Operating Mode
            0x01,   # execute Recenter once
        )
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = command_generator(
            cmd_type='03',  # command frame / Reply is reqired after data is sent.
            cmd_set='0E',
            cmd_id='0E',
            data=cmd_data
        )
        self.send_can_message(cmd)
        self.cmd_list_.append(cmd)
            
    def move_to(
        self, 
        dyaw: float = 0, 
        dpitch: float = 0, 
        droll: float = 0, 
        time_ms: int = 0
    ):
        hex_data = struct.pack(
            '<3h2B',    # format: https://docs.python.org/3/library/struct.html#format-strings
            int(dyaw * 10),
            int(droll * 10),
            int(dpitch * 10),
            0x00, # ctrl_byte,
            np.uint8(time_ms / 100), # time_for_action
        )
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = command_generator(
            cmd_type='03',  # command frame / Reply is reqired after data is sent.
            cmd_set='0E',
            cmd_id='00',
            data=cmd_data
        )
        self.send_can_message(cmd)
        self.cmd_list_.append(cmd)
        
    def _process_cmd(self, data):
        cmd_type = data[3]
        cmd_set = 0 # command set (CmdSet)
        cmd_id = 0  # command code (CmdID)
        is_ok = False
        
        # If it is a reply frame, need to check the corresponding send command
        if cmd_type == 0x20:
            for i, cmd in enumerate(self.cmd_list_):
                if data[8:10] == cmd[8:10]: # compare the serial numbers
                    cmd_set = cmd[12]
                    cmd_id = cmd[13]
                    self.cmd_list_.pop(i)   # remove the matched element
                    is_ok = True
                    break
        else:
            cmd_set = cmd[12]
            cmd_id = cmd[13]
            is_ok = True
            
        if is_ok:
            self.get_logger().debug(f'CmdSet: {cmd_set}\tCmdID: {cmd_id}')
            cmd_key = bytearray2value([cmd_set, cmd_id])
            if cmd_key == 0x000E:
                self.get_logger().info('get posControl request.')
            elif cmd_key == 0x020E:
                return_code = data[14]
                data_type = data[15]
                
                # return code
                serial = bytearray2value(data[8:10])
                if return_code == 0x00:
                    self.get_logger().debug(f'[{serial}] Command execuition succeeds.')
                elif return_code == 0x01:
                    self.get_logger().error(f'[{serial}] Command parse error.')
                    return None
                elif return_code == 0x02:
                    self.get_logger().error(f'[{serial}] Command execuition fails.')
                    return None
                elif return_code == 0xFF:
                    self.get_logger().error(f'[{serial}] Undefined error.')
                    return None

                yaw = bytearray2value(data[16:18], signed=True) * 0.1
                roll = bytearray2value(data[18:20], signed=True) * 0.1
                pitch = bytearray2value(data[20:22], signed=True) * 0.1
                
                # data type
                if data_type == 0x00:
                    self.get_logger().error('Data is not ready.')
                elif data_type == 0x01:
                    data_type = 'attitude_angle'
                    self.get_logger().debug(f'Attitude angle: {yaw}(yaw) {pitch}(pitch) {roll}(roll)')
                elif data_type == 0x02:
                    data_type = 'joint_angle'
                    self.get_logger().debug(f'Joint angle: {yaw}(yaw) {pitch}(pitch) {roll}(roll)')
                else:
                    self.get_logger().error('Undefined data type: 0x{:02X}'.format(data_type))

                return {
                    'type': data_type,
                    'yaw': yaw,
                    'pitch': pitch,
                    'roll': roll,
                }
            else:
                self.get_logger().info('get unknown request ({:04X}).'.format(cmd_key))

        return None
        
    def dji_command_parser(self, frame: bytearray) -> Any:
        # SOF
        if not (len(frame) > 12 and frame[0] == 0xAA):
            return None

        pack_len = frame[1] # Length
        version = frame[2]    # Version number
        cmd_type = frame[3]   # Command Type
        enc = frame[4]    # Encrypting
        res = frame[5:8]  # Reserved byte segment
        seq = frame[8:10] # Serial number
        crc16 = frame[10:12]  # Frame header check
        data = frame[12:-4]   # data
        crc32 = frame[-4:]    # Frame check (the entire frame) 

        step = 0            
        v1_pack_list = []
        for i in range(0, len(frame)):
            if step == 0:
                if frame[i] == 0xAA:
                    v1_pack_list.append(frame[i])
                    step = 1
            elif step == 1:
                pack_len = frame[i]
                v1_pack_list.append(frame[i])
                step = 2
            elif step == 2:
                version = frame[i]
                v1_pack_list.append(frame[i])
                step = 3
            elif step == 3:
                v1_pack_list.append(frame[i])
                if len(v1_pack_list) == 12:
                    if _check_head_crc(v1_pack_list):
                        step = 4
                    else:
                        step = 0
                        v1_pack_list.clear()
            elif step == 4:
                v1_pack_list.append(frame[i])
                if len(v1_pack_list) == pack_len:
                    step = 0
                    if _check_pack_crc(v1_pack_list):
                        result = self._process_cmd(v1_pack_list)
                        if result is not None:
                            return result
                    v1_pack_list.clear()
            else:
                step = 0
                v1_pack_list.clear()
                
        self.get_logger().debug(f'dji_command_parser() failed for {bytearray2string(frame)}')
        return None
    
    def wait_reply_frame(self, timeout_duration):
        dji_cmd = []
        length = -1 # the excepted command length
        stime = time.time()
        while True:
            msg = self.bus_.recv(timeout=self.timeout_)
            if msg is None:
                break
            
            if msg.data[0] == 0xAA:
                length = msg.data[1]
                dji_cmd = msg.data
            elif len(dji_cmd) > 0:
                dji_cmd += msg.data
                if len(dji_cmd) >= length > 0:
                    dji_cmd = dji_cmd[:length]
                    result = self.dji_command_parser(dji_cmd)
                    if result is not None and result['type'] == 'attitude_angle':
                        return result
        
            if len(self.cmd_list_) == 0 or time.time() - stime > timeout_duration:
                break
        return None
   
    def loop(self):
        self.bus_.flush_tx_buffer()
        
        # get current position
        hex_data = struct.pack(
            '<1B',    # format: https://docs.python.org/3/library/struct.html#format-strings
            0x01
        )
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = command_generator(
            cmd_type='03',
            cmd_set='0E',
            cmd_id='02',
            data=cmd_data
        )
        self.send_can_message(cmd)
        self.cmd_list_.clear()
        self.cmd_list_.append(cmd)

        r = self.wait_reply_frame(timeout_duration=0.1)
        if r is not None and r['type'] == 'attitude_angle':
            msg = DjiRsStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.yaw = r['yaw']
            msg.pitch = r['pitch']
            msg.roll = r['roll']
            self.rs3_status_publisher_.publish(msg)
        
        self.get_logger().debug('End of Loop.')


def main(args=None):
    rclpy.init(args=args)
    node = DjiRs3Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()