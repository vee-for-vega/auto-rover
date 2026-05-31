#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher() : Node("odom_publisher"), x_(0.0), y_(0.0), theta_(0.0)
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/raw", 10);

    // Latched QoS: a late-joining publisher of /rover/armed still reaches us,
    // and we still get the last armed state even if we start after the service.
    rclcpp::QoS armed_qos(1);
    armed_qos.transient_local();
    armed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/rover/armed",
      armed_qos,
      std::bind(&OdomPublisher::armed_callback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(50ms, std::bind(&OdomPublisher::publish_odom, this));
    RCLCPP_INFO(this->get_logger(), "Odometry publisher started (DISARMED — waiting to be armed)");
  }

private:
  void publish_odom()
  {
    double dt = 0.05;
    // Disarmed = freeze motion (zero velocity), but KEEP publishing.
    // "Stop" means commanding zeros, not going silent. Silence is the
    // failure signal the Module 4 watchdog listens for — never overload it.
    double linear_vel = armed_ ? 0.5 : 0.0;
    double angular_vel = armed_ ? 0.1 : 0.0;

    theta_ += angular_vel * dt;
    x_ += linear_vel * std::cos(theta_) * dt;
    y_ += linear_vel * std::sin(theta_) * dt;

    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

    msg.twist.twist.linear.x = linear_vel;
    msg.twist.twist.angular.z = angular_vel;

    // x and y position variance — EKF will read this in Module 3
    msg.pose.covariance[0] = 0.01;  // x variance
    msg.pose.covariance[7] = 0.01;  // y variance

    publisher_->publish(msg);
  }

  void armed_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data != armed_) {
      RCLCPP_INFO(this->get_logger(),
        "Armed state changed -> %s", msg->data ? "ARMED (moving)" : "DISARMED (frozen)");
    }
    armed_ = msg->data;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_, y_, theta_;
  bool armed_ = false;  // safe default: do not move until explicitly armed
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}