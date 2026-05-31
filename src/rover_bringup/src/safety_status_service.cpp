#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>

// We're reusing SetBool as a stand-in:
// Request:  bool data  (true = arm the rover, false = disarm)
// Response: bool success, string message

class SafetyStatusService : public rclcpp::Node
{
public:
  SafetyStatusService() : Node("safety_status_service"), armed_(false)
  {
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "/rover/set_armed",
      std::bind(&SafetyStatusService::handle_arm, this,
        std::placeholders::_1, std::placeholders::_2)
    );

    // Latched QoS so any node that subscribes later still receives the
    // current armed state — operational-state topics must be transient_local.
    rclcpp::QoS armed_qos(1);
    armed_qos.transient_local();
    armed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/rover/armed", armed_qos);

    // Publish the initial (safe) state so subscribers know we start DISARMED.
    publish_armed();

    RCLCPP_INFO(this->get_logger(), "Safety status service ready (DISARMED)");
  }

private:
  void handle_arm(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    armed_ = request->data;
    publish_armed();  // <-- the wire: propagate the new state onto the graph
    response->success = true;
    response->message = armed_ ? "Rover ARMED — ready to move" : "Rover DISARMED — motors locked";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void publish_armed()
  {
    std_msgs::msg::Bool msg;
    msg.data = armed_;
    armed_pub_->publish(msg);
  }

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  bool armed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyStatusService>());
  rclcpp::shutdown();
  return 0;
}