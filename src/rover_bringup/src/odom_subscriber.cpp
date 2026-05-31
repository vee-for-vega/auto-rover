#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomSubscriber : public rclcpp::Node
{
public:
  OdomSubscriber() : Node("odom_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/raw",
      10,
      std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1)
    );
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Position: (%.3f, %.3f) | Heading: %.3f rad",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      2.0 * std::asin(msg->pose.pose.orientation.z)
    );
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}