#!/usr/bin/python3
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from wildpose_interfaces.msg import MotorStatus
 
import math


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
        'dpad': [-msg.axes[6], msg.axes[7]],
    }

    return buttons, axes


class MotorControlNode(Node):
    def __init__(self, n_motor=3):
        super().__init__('motor_control_node')
        
        # Motor parameters
        self.n_motor_ = n_motor
        self.motor_speed_ = 200
        self.revolutions_ = [0.0] * self.n_motor_   # the revolution of the gearmotor output 
        self.pluse_counters_ = [0] * self.n_motor_
        self.motor_speeds_ = [0] * self.n_motor_
        self._update_motor_flag = False
        self._reset_motor_flag = False
        self.motor_status_publishers_ = [
            self.create_publisher(MotorStatus, f'motor{i+1}', 10)
            for i in range(self.n_motor_)
        ]

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
        self.reset_motor()

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
        
        # motor 1
        if axes['dpad'][0] != 0:    
            self.motor_speeds_[0] = int(math.copysign(self.motor_speed_, axes['dpad'][0]))
            self._update_motor_flag = True
        else:
            self.motor_speeds_[0] = 0
            self._update_motor_flag = True

        # motor 2
        if axes['dpad'][1] != 0:    
            self.motor_speeds_[1] = int(math.copysign(self.motor_speed_, axes['dpad'][1]))
            self._update_motor_flag = True
        else:
            self.motor_speeds_[1] = 0
            self._update_motor_flag = True

        # motor 3
        if axes['joy_left'][1] != 0:    
            self.motor_speeds_[2] = int(math.copysign(self.motor_speed_, axes['joy_left'][1]))
            self._update_motor_flag = True
        else:
            self.motor_speeds_[2] = 0
            self._update_motor_flag = True
            
        # reset
        if buttons['Logitech'] == 1:
            self._reset_motor_flag = True
            
    def reset_motor(self):
        self.serial_port_.write('reset\n'.encode())
        self.get_logger().info(f'Reset the motor status.')

    def check_motor_state(self,):
        # set motor status
        if self._update_motor_flag:
            for i in range(self.n_motor_):
                self.serial_port_.write(f't{i}{self.motor_speeds_[i]}\n'.encode())
            self.get_logger().info(f'set motor speed: {self.motor_speeds_}')
            self._update_motor_flag = False
            
        if self._reset_motor_flag:
            self.reset_motor()
            self._reset_motor_flag = False
        
        # get motor status
        while self.serial_port_.inWaiting() > 0:
            # get serial signals
            data = self.serial_port_.readline()
            if data is None:
                return None
            # convert the serial
            data = data.decode()
            if len(data) < 3:
                self.get_logger().error(f'Undefined command: {data}')
                return None
            # parser
            sign = data[0]
            motor_id = int(data[1])
            value = data[2:]
            if sign == 'r': # get the revolution of the gearmotor output
                try:
                    self.revolutions_[motor_id] = float(value)
                except ValueError:
                    self.get_logger().error(f'Failed to covnert "{value}" a float value.')
            elif sign == 'p':   # get the pulse count
                try:
                    self.pluse_counters_[motor_id] = int(value)
                except ValueError:
                    self.get_logger().error(f'Failed to covnert "{value}" an int value.')
            else:
                self.get_logger().error(f'Undefined command: {data}')
                    
        for i in range(self.n_motor_):
            msg = MotorStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pulse_count = self.pluse_counters_[i]
            msg.revolution = self.revolutions_[i]
            self.motor_status_publishers_[i].publish(msg)

    def __del__(self):
        self.serial_port_.close()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
