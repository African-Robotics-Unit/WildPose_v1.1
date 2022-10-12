#!/usr/bin/python3
import serial
import rclpy
from rclpy.node import Node


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Serial setting
        self.serial_port_ = serial.Serial(
            port="/dev/ttyTHS0",
            baudrate=115200,    # it should be 115200 bps for Jetson AGX Orin
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5
        )
        self.serial_port_.flushInput()
        
        # timer
        self.motor_state_timer_ = self.create_timer(0.1, self.check_motor_state)
        
    def check_motor_state(self,):
        while self.serial_port_.inWaiting() > 0:
            data = self.serial_port_.readline()
            if data is not None:
                self.get_logger().info(data.decode())
            # # if we get a carriage return, add a line feed too
            # # \r is a carriage return; \n is a line feed
            # # This is to help the tty program on the other end 
            # # Windows is \r\n for carriage return, line feed
            # # Macintosh and Linux use \n
            # if data == "\r".encode():
            #     # For Windows boxen on the other end
            #     self.serial_port_.write("\n".encode())
            
    def __del__(self):
        self.serial_port_.close()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
