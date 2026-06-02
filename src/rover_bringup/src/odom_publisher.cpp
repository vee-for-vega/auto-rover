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
    // Tunable motion parameters: override at launch
    //   --ros-args -p linear_vel:=1.0
    // or live at runtime
    //   ros2 param set /odom_publisher linear_vel 1.5
    this->declare_parameter<double>("linear_vel", 0.5);
    this->declare_parameter<double>("angular_vel", 0.1);

    // Publish rate is applied once at startup (the timer is built with this period).
    // Changing it live would need an on-set callback that recreates the timer.
    this->declare_parameter<double>("publish_rate_hz", 20.0);
    dt_ = 1.0 / this->get_parameter("publish_rate_hz").as_double();

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

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(dt_));
    timer_ = this->create_wall_timer(period, std::bind(&OdomPublisher::publish_odom, this));
    RCLCPP_INFO(this->get_logger(), "Odometry publisher started (DISARMED — waiting to be armed)");
  }

private:
  void publish_odom()
  {
    double dt = dt_;
    // Disarmed = freeze motion (zero velocity), but KEEP publishing.
    // "Stop" means commanding zeros, not going silent. Silence is the
    // failure signal the Module 4 watchdog listens for — never overload it.
    double linear_vel  = armed_ ? this->get_parameter("linear_vel").as_double()  : 0.0;
    double angular_vel = armed_ ? this->get_parameter("angular_vel").as_double() : 0.0;

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
  double dt_ = 0.05;  // seconds between publishes; set from publish_rate_hz at startup
  bool armed_ = false;  // safe default: do not move until explicitly armed
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}