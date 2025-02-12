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

        self.allocator_bow = rf.MinimumMagnitudeAndAzimuth(
            actuators=ma1_actuators.bow_actuators(),
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

        self.allocator_bow.compute_configuration_matrix()

        self.allocator_stern = rf.MinimumMagnitudeAndAzimuth(
            actuators=ma1_actuators.stern_actuators(),
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

        self.allocator_stern.compute_configuration_matrix()

        self.wrench_sub_ = self.create_subscription(
            Wrench, 'ma1/tau', self.wrench_callback, 10)
        
        self.allocated_tau_pub_ = self.create_publisher(
            Wrench, 'ma1/allocated_tau', 10)
        
        self.thrust_pub_ = self.create_publisher(
            Float64MultiArray, '/ma1/thrusters', 10)
        
        self.azi_pub_ = self.create_publisher(
            JointState, '/ma1/servos', 10)
        
        self.mode = self.declare_parameter('mode', '_').get_parameter_value().string_value

        self.get_logger().info(f'Mode: {self.mode}')
        
        self.get_logger().info('Thrust Allocation Node has been initialized')

    def wrench_callback(self, msg: Wrench):
        if self.mode == 'normal':
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

            allocated_tau = self.allocator_.allocated.flatten()

        elif self.mode == 'hybrid':

            stern_tau = np.array([[msg.force.x, msg.force.y / 2, 0, 0, 0, 0]]).T
            bow_tau = np.array([[0, msg.force.y / 2, 0, 0, 0, msg.torque.z]]).T

            stern_forces = self.allocator_stern.allocate(stern_tau)[0].flatten()
            bow_forces = self.allocator_bow.allocate(bow_tau)[0].flatten()

            thruster_1_thrust = np.linalg.norm(bow_forces[0:2])
            thruster_2_thrust = np.linalg.norm(bow_forces[2:4])
            thruster_3_thrust = np.linalg.norm(stern_forces[0:2])
            thruster_4_thrust = np.linalg.norm(stern_forces[2:4])

            angle_1 = np.arctan2(bow_forces[1], bow_forces[0])
            angle_2 = np.arctan2(bow_forces[3], bow_forces[2])
            angle_3 = np.arctan2(stern_forces[1], stern_forces[0])
            angle_4 = np.arctan2(stern_forces[3], stern_forces[2])

            allocated_tau = self.allocator_bow.allocated.flatten() + self.allocator_stern.allocated.flatten()

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

        allocated_tau_msg = Wrench()
        allocated_tau_msg.force.x = allocated_tau[0]
        allocated_tau_msg.force.y = allocated_tau[1]
        allocated_tau_msg.torque.z = allocated_tau[5]
        self.allocated_tau_pub_.publish(allocated_tau_msg)
        
def main(args=None):
    rclpy.init(args=args)

    thrust_allocation_node = ThrustAllocationNode()

    rclpy.spin(thrust_allocation_node)

    thrust_allocation_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()