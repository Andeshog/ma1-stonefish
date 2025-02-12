#ifndef THRUST_ALLOCATOR_NODE_HPP
#define THRUST_ALLOCATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ma1_qp/thrust_allocator.hpp"

class ThrustAllocatorNode : public rclcpp::Node
{
public:
  ThrustAllocatorNode();

private:
  void tauCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr tau_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angles_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_pub_;

  // Initialize the thrust allocator with vessel dimensions and max thrust.
  ThrustAllocator thrust_allocator_;
};

#endif // THRUST_ALLOCATOR_NODE_HPP
