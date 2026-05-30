#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

// NOTE: Assumes single-threaded executor. Loading into a MultiThreadedExecutor
// would require mutex protection on heartbeat_ok_, enabled_, and channel buffers.

namespace command_gate {

struct Channel {
  std::string input_topic, output_topic, message_type, fallback_topic;
  rclcpp::GenericSubscription::SharedPtr input_sub;
  rclcpp::GenericPublisher::SharedPtr output_pub;
  rclcpp::GenericSubscription::SharedPtr fallback_sub;
  std::shared_ptr<rclcpp::SerializedMessage> last_fallback_msg;
  std::shared_ptr<rclcpp::SerializedMessage> zero_msg;
  bool warned_drop{false};
};

class CommandGateNode : public rclcpp::Node
{
public:
  explicit CommandGateNode(const rclcpp::NodeOptions & options);

private:
  bool enabled_{false};
  bool heartbeat_ok_{false};
  bool require_heartbeat_{true};
  bool require_enable_{false};
  double heartbeat_timeout_{1.0};
  rclcpp::Time last_heartbeat_time_;
  std::vector<Channel> channels_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_srv_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  void declareAndGetParams();
  void setupChannels();
  void setupHeartbeat();
  void setupEnable();

  std::shared_ptr<rclcpp::SerializedMessage> makeZeroMessage(const std::string & type);
  bool isGateOpen() const;
  std::string openReason(const std::string & trigger) const;
  void onGateStateChange(bool was_open, bool now_open, const std::string & reason);
  void publishFallbackOrZero(Channel & ch);

  void onHeartbeatCb(const std_msgs::msg::Empty::SharedPtr msg);
  void heartbeatCheckTimerCb();
  void onEnableTopicCb(const std_msgs::msg::Bool::SharedPtr msg);
  void onSetEnabledCb(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
};

}  // namespace command_gate
