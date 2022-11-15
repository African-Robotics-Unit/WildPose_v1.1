#!/usr/bin/python3
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


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

        # Logitech F510
        self.joy_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # timer
        self.motor_state_timer_ = self.create_timer(0.1, self.check_motor_state)

    def joy_callback(self, msg):
        buttons, axes = joymsg2f510(msg)

    def check_motor_state(self,):
        self.serial_port_.write('test'.encode())
        while self.serial_port_.inWaiting() > 0:
            data = self.serial_port_.readline()
            if data is not None:
                self.get_logger().info(data.decode())

    def __del__(self):
        self.serial_port_.close()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
