import skadipy as sk
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench

ma_bow_port = sk.actuator.Fixed(
    position=sk.toolbox.Point([1.8, -0.8, 0.0]),
    orientation=sk.toolbox.Quaternion(
        axis=(0.0, 0.0, 1.0), angle=(-np.pi / 4)
    )
)

ma_bow_starboard = sk.actuator.Fixed(
    position=sk.toolbox.Point([1.8, 0.8, 0.0]),
    orientation=sk.toolbox.Quaternion(
        axis=(0.0, 0.0, 1.0), angle=(np.pi / 4)
    )
)

ma_stern_starboard = sk.actuator.Fixed(
    position=sk.toolbox.Point([-1.8, 0.8, 0.0]),
    orientation=sk.toolbox.Quaternion(
        axis=(0.0, 0.0, 1.0), angle=(3 * np.pi / 4)
    )
)

ma_stern_port = sk.actuator.Fixed(
    position=sk.toolbox.Point([-1.8, -0.8, 0.0]),
    orientation=sk.toolbox.Quaternion(
        axis=(0.0, 0.0, 1.0), angle=(-3 * np.pi / 4)
    )
)

class ThrustAllocationNode(Node):
    def __init__(self):
        super().__init__('thrust_allocation_node')
        self.allocator_ = sk.allocator.QuadraticProgramming(
            actuators=[
                ma_bow_port,
                ma_bow_starboard,
                ma_stern_starboard,
                ma_stern_port,
            ],
            force_torque_components=[
                sk.allocator.ForceTorqueComponent.X,
                sk.allocator.ForceTorqueComponent.Y,
                sk.allocator.ForceTorqueComponent.N,
            ]
        )

        self.allocator_.compute_configuration_matrix()

        self.wrench_sub_ = self.create_subscription(
            Wrench, 'ma1/tau', self.wrench_callback, 10)
        
        self.thrust_pub_ = self.create_publisher(
            Float64MultiArray, '/ma1/thrusters', 10)
        
        self.get_logger().info('Thrust Allocation Node has been initialized')

    def wrench_callback(self, msg: Wrench):
        tau_list = [msg.force.x, msg.force.y, 0, 0, 0, msg.torque.z]
        tau = np.array([tau_list]).T
        thrust = self.allocator_.allocate(tau)[0]

        thrust_msg = Float64MultiArray()
        thrust_msg.data = thrust.tolist()
        self.thrust_pub_.publish(thrust_msg)
        


def main(args=None):
    rclpy.init(args=args)

    thrust_allocation_node = ThrustAllocationNode()

    rclpy.spin(thrust_allocation_node)

    thrust_allocation_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()