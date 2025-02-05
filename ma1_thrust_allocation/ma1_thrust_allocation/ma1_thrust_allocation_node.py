#!/usr/bin/env python3

import skadipy as sk
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench
from ma1_thrust_allocation.ma1_actuators import Actuators
from skadipy.allocator import reference_filters as rf
from sensor_msgs.msg import JointState

ma1_actuators = Actuators()

class ThrustAllocationNode(Node):
    def __init__(self):
        super().__init__('thrust_allocation_node')
        self.allocator_ = rf.MinimumMagnitudeAndAzimuth(
            actuators=ma1_actuators.all_actuators(),
            force_torque_components=[
                sk.allocator.ForceTorqueComponent.X,
                sk.allocator.ForceTorqueComponent.Y,
                sk.allocator.ForceTorqueComponent.N,
            ],
            gamma=0.2,
            mu=10,
            rho=0.1,
            time_step=0.065, # Publish rate of joy
            control_barrier_function=sk.safety.ControlBarrierFunctionType.ABSOLUTE
        )

        self.allocator_.compute_configuration_matrix()

        self.wrench_sub_ = self.create_subscription(
            Wrench, 'ma1/tau', self.wrench_callback, 10)
        
        self.thrust_pub_ = self.create_publisher(
            Float64MultiArray, '/ma1/thrusters', 10)
        
        self.azi_pub_ = self.create_publisher(
            JointState, '/ma1/servos', 10)
        
        self.get_logger().info('Thrust Allocation Node has been initialized')

    def wrench_callback(self, msg: Wrench):
        tau_list = [msg.force.x, msg.force.y, 0, 0, 0, msg.torque.z]
        tau = np.array([tau_list]).T
        forces = self.allocator_.allocate(tau)[0].flatten()

        thruster_1_thrust = np.linalg.norm(forces[0:2])
        thruster_2_thrust = np.linalg.norm(forces[2:4])
        thruster_3_thrust = np.linalg.norm(forces[4:6])
        thruster_4_thrust = np.linalg.norm(forces[6:8])

        angle_1 = np.arctan2(forces[1], forces[0])
        angle_2 = np.arctan2(forces[3], forces[2])
        angle_3 = np.arctan2(forces[5], forces[4])
        angle_4 = np.arctan2(forces[7], forces[6])

        thrust = [thruster_1_thrust, thruster_2_thrust, thruster_3_thrust, thruster_4_thrust]
        azi_names = ['mA1/azimuth_1_joint', 'mA1/azimuth_2_joint', 'mA1/azimuth_3_joint', 'mA1/azimuth_4_joint']
        angles = [angle_1, angle_2, angle_3, angle_4]
        
        thrust_msg = Float64MultiArray()
        thrust_msg.data = thrust
        self.thrust_pub_.publish(thrust_msg)

        azi_msg = JointState()
        azi_msg.name = azi_names
        azi_msg.position = angles
        self.azi_pub_.publish(azi_msg)
        


def main(args=None):
    rclpy.init(args=args)

    thrust_allocation_node = ThrustAllocationNode()

    rclpy.spin(thrust_allocation_node)

    thrust_allocation_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()