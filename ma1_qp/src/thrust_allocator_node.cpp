#include "ma1_qp/thrust_allocator_node.hpp"
#include <vector>

ThrustAllocatorNode::ThrustAllocatorNode()
  : Node("thrust_allocator_node"),
    thrust_allocator_(100.0)
{
    tau_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/ma1/tau", 10,
        std::bind(&ThrustAllocatorNode::tauCallback, this, std::placeholders::_1));

    angles_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/ma1/servos", 10);
    forces_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ma1/thrusters", 10);
}

void ThrustAllocatorNode::tauCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    // Extract the desired forces and moment.
    double Fx = msg->force.x;
    double Fy = msg->force.y;
    double Mz = msg->torque.z;

    // Compute the thrust allocation.
    auto commands = thrust_allocator_.allocate(Fx, Fy, Mz);

    std::vector<double> angles(commands.size());
    std_msgs::msg::Float64MultiArray forces_msg;
    forces_msg.data.resize(commands.size());

    for (size_t i = 0; i < commands.size(); ++i)
    {
        angles[i] = commands[i].azimuth;
        forces_msg.data[i] = commands[i].magnitude;
    }
    std::vector<std::string> names = {"mA1/azimuth_1_joint", "mA1/azimuth_2_joint", "mA1/azimuth_3_joint", "mA1/azimuth_4_joint"};
    sensor_msgs::msg::JointState angles_msg;
    angles_msg.name = names;
    angles_msg.position = angles;
    angles_pub_->publish(angles_msg);
    forces_pub_->publish(forces_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrustAllocatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}