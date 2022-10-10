#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import can
import struct
from .SDKCRC import calc_crc


BITRATE = 1000000
Seq_Init_Data = 0x0002

# 获取sequence序列
def seq_num():
	"""
	通过全局变量 SeqInit_Data 来获取下一个的 sequence number
	"""
	global Seq_Init_Data
	if Seq_Init_Data >= 0xFFFD:
		Seq_Init_Data = 0x0002
	Seq_Init_Data += 1
	seq_str = "%04x" % Seq_Init_Data
	return seq_str[2:] + ":" + seq_str[0:2]


def combine(cmd_type, cmd_set, cmd_id, data):
	"""
	拼接OSDK命令
	data为空， 则 data = ""
	"""
	if data == "":
		tmp_cmd = "{prefix}" + ":{cmd_set}:{cmd_id}".format(cmd_set=cmd_set, cmd_id=cmd_id)
	else:
		tmp_cmd = "{prefix}" + ":{cmd_set}:{cmd_id}:{data}".format(cmd_set=cmd_set, cmd_id=cmd_id,data=data)

	cmd_length = len(tmp_cmd.split(":")) + 15 #总的数据包长度
	seqnum = seq_num()

	cmd_prefix="AA" + ":" + ("%04x"%(cmd_length))[2:4] + ":" + ("%04x"%(cmd_length))[0:2] \
			   + ":{cmd_type}:{enc:02x}:{res1:02x}:{res2:02x}:{res3:02x}:{seqnum}".format(cmd_type=cmd_type, enc=0x00,res1=0x00, res2=0x00, res3=0x00,seqnum=seqnum)
	#计算CRC16
	crc16_val = calc_crc(cmd_prefix, 16)
	cmd_prefix = cmd_prefix + ":" + crc16_val

	cmd = tmp_cmd.format(prefix=cmd_prefix)
	crc32_val = calc_crc(cmd, 32)
	cmd = cmd + ":" + crc32_val

	return cmd


class DjiRs3Node(Node):
    def __init__(self):
        super().__init__("dji_rs3_node")
        
        # Node parameters
        self.declare_parameter("channel", "can0")
        self.channel_ = self.get_parameter("channel").value
        
        self.bus = can.interface.Bus(
            bustype='socketcan', 
            channel=self.channel_, 
            bitrate=BITRATE
        )
        
        self.timer_ = self.create_timer(1.0, self.send_data)
        
        self.get_logger().info("dji_rs3_node started.")
        
    def get_data(self):
        msg = self.bus.recv()
        if msg is not None:
            self.get_logger().info(f'{msg}')
            self.get_logger().info(f'{type(msg)}\t{msg.dlc}\t{msg.data}')
            
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
        # cmd = combine(cmd_type='03', cmd_set='0E', cmd_id='00', data=cmd_data)
        # self.get_logger().info(f'cmd: {cmd}')
        
        msg = can.Message(
            arbitration_id=0x01, 
            is_extended_id=True,
            is_rx=False,
            data=bytearray([0xAA, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x7A, 0x1E, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x03, 0x01, 0x14, 0x88, 0x08, 0x1C, 0xA9])
        )
        self.get_logger().info(f'msg: {msg}')
        self.bus.send(msg, timeout=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = DjiRs3Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()