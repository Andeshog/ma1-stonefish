#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Wrench

class JoystickInterface(Node):
    def __init__(self):
        super().__init__('ma1_joystick_interface_node')

        self.joy_sub_ = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.tau_pub_ = self.create_publisher(
            Wrench, 'ma1/tau', 10)
        
        self.init_gains()

    def init_gains(self):
        self.declare_parameters('', [
            ('surge_gain', 1.0),
            ('sway_gain', 1.0),
            ('yaw_gain', 1.0)
        ])

        self.surge_gain_ = self.get_parameter('surge_gain').value
        self.sway_gain_ = self.get_parameter('sway_gain').value
        self.yaw_gain_ = self.get_parameter('yaw_gain').value
        
    def joy_callback(self, msg: Joy):
        left_vertical = msg.axes[1]
        left_horizontal = msg.axes[0]
        right_horizontal = msg.axes[3]

        tau = Wrench()
        tau.force.x = self.surge_gain_ * left_vertical
        tau.force.y = self.sway_gain_ * left_horizontal
        tau.torque.z = self.yaw_gain_ * right_horizontal

        self.tau_pub_.publish(tau)
        


def main(args=None):
    rclpy.init(args=args)

    joystick_interface_node = JoystickInterface()

    rclpy.spin(joystick_interface_node)

    joystick_interface_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()