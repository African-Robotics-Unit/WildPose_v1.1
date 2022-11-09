#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import can
import struct
import numpy as np
from typing import List
from .protocol.sdk import CmdCombine


BITRATE = 1000000
CAN_LENQ = 8
Seq_Init_Data = 0x0002


def command_generator(cmd_type: str, cmd_set: str, cmd_id: str, data: str) -> bytearray:
    dji_cmd = CmdCombine.combine(cmd_type=cmd_type, cmd_set=cmd_set, cmd_id=cmd_id, data=data)
    return [int(b, 16) for b in dji_cmd.split(':')]


class DjiRs3Node(Node):
    def __init__(self):
        super().__init__("dji_rs3_node")

        # Node parameters
        self.declare_parameter("channel", "can0")
        self.channel_ = self.get_parameter("channel").value

        self.bus_ = can.interface.Bus(
            bustype='socketcan',
            channel=self.channel_,
            bitrate=BITRATE
        )
        self.send_id_ = 0x223
        self.rev_id_ = 0x222

        self.send_data()
        self.timer_ = self.create_timer(1.0, self.loop)

        self.get_logger().info("dji_rs3_node started.")

    def get_data(self):
        msg = self.bus_.recv()
        if msg is not None:
            self.get_logger().info(f'{msg}')
            self.get_logger().info(f'{type(msg)}\t{msg.dlc}\t{msg.data}')
            
    def send_can_message(self, cmd: List):
        for i in range(0, len(cmd), CAN_LENQ):
            print(cmd[i:i+CAN_LENQ])
            data = bytearray(cmd[i:i+CAN_LENQ])
            msg = can.Message(
                arbitration_id=self.send_id_,
                is_extended_id=False,
                is_rx=False,
                data=data
            )
            try:
                self.bus_.send(msg)
                self.get_logger().info(f'Message sent {data} on {self.bus_.channel_info}')
            except can.CanError:
                self.get_logger().error("Faild to send a CAN message.")
            
    def move_to(self, yaw, pitch, roll, time_ms):
        hex_data = struct.pack(
            '<3h2B',    # format: https://docs.python.org/3/library/struct.html#format-strings
            int(yaw * 10),
            int(roll * 10),
            int(pitch * 10),
            0x00, # ctrl_byte,
            np.uint8(time_ms / 100), # time_for_action
        )
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = command_generator(
            cmd_type='03',
            cmd_set='0E',
            cmd_id='00',
            data=cmd_data
        )
        self.send_can_message(cmd)

    def send_data(self):
        # hex_data = struct.pack(
        #     '<3h2B',
        #     0, # yaw,
        #     0, # roll,
        #     90 * 10, # pitch,
        #     0x01, # ctrl_byte,
        #     0x14, # time_for_action
        # )
        # pack_data = ['{:02X}'.format(i) for i in hex_data]
        # cmd_data = ':'.join(pack_data)
        # cmd = CmdCombine.combine(cmd_type='03', cmd_set='0E', cmd_id='00', data=cmd_data)
        # self.get_logger().info(f'cmd: {cmd}')
            
        self.move_to(yaw=90, pitch=0, roll=0, time_ms=500)

        # self.get_logger().info(f'msg: {msg}')
        # self.bus_.send(msg, timeout=0.5)
        
    def loop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DjiRs3Node()
    # rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()