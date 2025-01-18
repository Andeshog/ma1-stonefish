#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/wrench.hpp"

class ThrusterController : public rclcpp::Node
{
public:
    ThrusterController() : Node("thruster_controller")
    {
        // Subscriber to joystick inputs
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ThrusterController::joyCallback, this, std::placeholders::_1));
        
        // Publisher for thruster setpoints
        thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ma1/thrusters", 10);
        wrench_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ma1/tau", 10);
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        // Assuming left joystick vertical (axes[1]) -> surge
        // Assuming left joystick horizontal (axes[0]) -> sway
        // Assuming right joystick horizontal (axes[3]) -> yaw
        int scale = 350;
        int yaw_scale = 750;
        double surge = joy_msg->axes[1] * scale;
        double sway = joy_msg->axes[0] * scale;
        double yaw = joy_msg->axes[3] * yaw_scale;

        geometry_msgs::msg::Wrench wrench_msg;
        wrench_msg.force.x = surge;
        wrench_msg.force.y = sway;
        wrench_msg.torque.z = yaw;

        wrench_pub_->publish(wrench_msg);

        // Calculate thrust for each thruster based on surge, sway, and yaw
        std_msgs::msg::Float64MultiArray thruster_msg;
        thruster_msg.data.resize(4);

        // Thruster positions in X-configuration
        // Front-right thruster
        thruster_msg.data[1] = surge - sway - yaw;
        // Front-left thruster
        thruster_msg.data[0] = surge + sway + yaw;
        // Rear-left thruster
        thruster_msg.data[3] = surge - sway + yaw;
        // Rear-right thruster
        thruster_msg.data[2] = surge + sway - yaw;

        // Publish the thruster setpoints
        thruster_pub_->publish(thruster_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterController>());
    rclcpp::shutdown();
    return 0;
}
