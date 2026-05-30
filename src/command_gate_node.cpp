#include "command_gate/command_gate_node.hpp"

#include <chrono>
#include <string>

#include "rclcpp/serialization.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float64.hpp"

namespace command_gate {

CommandGateNode::CommandGateNode(const rclcpp::NodeOptions & options)
: Node("command_gate", options)
{
  declareAndGetParams();
  setupChannels();
  setupHeartbeat();
  setupEnable();

  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration<double>(heartbeat_timeout_ / 4.0),
    [this]() { heartbeatCheckTimerCb(); });

  RCLCPP_INFO(
    get_logger(),
    "CommandGateNode started: %zu channel(s), require_heartbeat=%s, require_enable=%s",
    channels_.size(),
    require_heartbeat_ ? "true" : "false",
    require_enable_ ? "true" : "false");
}

void CommandGateNode::declareAndGetParams()
{
  declare_parameter("num_channels", 1);
  declare_parameter("require_heartbeat", true);
  declare_parameter("heartbeat_timeout", 1.0);
  declare_parameter("heartbeat_topic", std::string("~/heartbeat"));
  declare_parameter("require_enable", false);
  declare_parameter("enable_topic", std::string("~/enable"));

  require_heartbeat_ = get_parameter("require_heartbeat").as_bool();
  heartbeat_timeout_ = get_parameter("heartbeat_timeout").as_double();
  require_enable_ = get_parameter("require_enable").as_bool();

  enabled_ = false;
  heartbeat_ok_ = false;
  // Initialize as timed-out so gate starts closed when require_heartbeat=true
  last_heartbeat_time_ =
    get_clock()->now() - rclcpp::Duration::from_seconds(heartbeat_timeout_ + 1.0);
}

void CommandGateNode::setupChannels()
{
  int num_channels = get_parameter("num_channels").as_int();

  for (int i = 0; i < num_channels; ++i) {
    std::string prefix = "channel_" + std::to_string(i);
    declare_parameter(prefix + ".input_topic", std::string(""));
    declare_parameter(prefix + ".output_topic", std::string(""));
    declare_parameter(prefix + ".message_type", std::string(""));
    declare_parameter(prefix + ".fallback_topic", std::string(""));

    Channel ch;
    ch.input_topic = get_parameter(prefix + ".input_topic").as_string();
    ch.output_topic = get_parameter(prefix + ".output_topic").as_string();
    ch.message_type = get_parameter(prefix + ".message_type").as_string();
    ch.fallback_topic = get_parameter(prefix + ".fallback_topic").as_string();

    if (ch.input_topic.empty() || ch.output_topic.empty() || ch.message_type.empty()) {
      RCLCPP_WARN(get_logger(), "Channel %d missing required parameters, skipping.", i);
      continue;
    }

    ch.output_pub = create_generic_publisher(ch.output_topic, ch.message_type, rclcpp::QoS(10));

    // idx is captured by value; lambdas index into channels_ after construction is done
    const int idx = static_cast<int>(channels_.size());

    ch.input_sub = create_generic_subscription(
      ch.input_topic, ch.message_type, rclcpp::QoS(10),
      [this, idx](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (isGateOpen()) {
          channels_[idx].output_pub->publish(*msg);
        }
      });

    if (!ch.fallback_topic.empty()) {
      ch.fallback_sub = create_generic_subscription(
        ch.fallback_topic, ch.message_type, rclcpp::QoS(1),
        [this, idx](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          // Deep copy — original shared_ptr is reclaimed after the callback returns
          channels_[idx].last_fallback_msg =
            std::make_shared<rclcpp::SerializedMessage>(*msg);
        });
    }

    ch.zero_msg = makeZeroMessage(ch.message_type);
    channels_.push_back(std::move(ch));
  }
}

void CommandGateNode::setupHeartbeat()
{
  std::string heartbeat_topic = get_parameter("heartbeat_topic").as_string();
  heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
    heartbeat_topic, rclcpp::QoS(10),
    [this](const std_msgs::msg::Empty::SharedPtr msg) { onHeartbeatCb(msg); });
}

void CommandGateNode::setupEnable()
{
  std::string enable_topic = get_parameter("enable_topic").as_string();
  enable_sub_ = create_subscription<std_msgs::msg::Bool>(
    enable_topic, rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) { onEnableTopicCb(msg); });

  set_enabled_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_enabled",
    [this](
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
      onSetEnabledCb(request, response);
    });
}

std::shared_ptr<rclcpp::SerializedMessage>
CommandGateNode::makeZeroMessage(const std::string & type)
{
  auto result = std::make_shared<rclcpp::SerializedMessage>();
#define TRY(MsgT, str) \
  if (type == (str)) { \
    rclcpp::Serialization<MsgT> s; MsgT m; s.serialize_message(&m, result.get()); \
    return result; \
  }
  TRY(geometry_msgs::msg::Twist,                  "geometry_msgs/msg/Twist")
  TRY(geometry_msgs::msg::TwistStamped,           "geometry_msgs/msg/TwistStamped")
  TRY(ackermann_msgs::msg::AckermannDrive,        "ackermann_msgs/msg/AckermannDrive")
  TRY(ackermann_msgs::msg::AckermannDriveStamped, "ackermann_msgs/msg/AckermannDriveStamped")
  TRY(std_msgs::msg::Float64,                     "std_msgs/msg/Float64")
#undef TRY
  RCLCPP_WARN(
    get_logger(), "Type '%s' not in known list — zero-init unavailable", type.c_str());
  return nullptr;
}

bool CommandGateNode::isGateOpen() const
{
  bool hb_ok = heartbeat_ok_ || !require_heartbeat_;
  bool en_ok = enabled_ || !require_enable_;
  return hb_ok && en_ok;
}

std::string CommandGateNode::openReason(const std::string & trigger) const
{
  if (require_heartbeat_ && require_enable_) {
    if (trigger.find("heartbeat") != std::string::npos) {
      return "heartbeat received and already enabled";
    }
    return "heartbeat alive and " + trigger;
  }
  return trigger;
}

void CommandGateNode::onGateStateChange(
  [[maybe_unused]] bool was_open, bool now_open, const std::string & reason)
{
  if (now_open) {
    RCLCPP_INFO(get_logger(), "Gate OPEN — %s", reason.c_str());
    for (auto & ch : channels_) {
      ch.warned_drop = false;
    }
  } else {
    RCLCPP_INFO(get_logger(), "Gate CLOSED — %s", reason.c_str());
    for (auto & ch : channels_) {
      publishFallbackOrZero(ch);
    }
  }
}

void CommandGateNode::publishFallbackOrZero(Channel & ch)
{
  if (ch.last_fallback_msg) {
    ch.output_pub->publish(*ch.last_fallback_msg);
  } else if (ch.zero_msg) {
    ch.output_pub->publish(*ch.zero_msg);
  } else if (!ch.warned_drop) {
    RCLCPP_WARN(
      get_logger(),
      "Gate closed on channel [%s]: no fallback or zero message available, dropping.",
      ch.output_topic.c_str());
    ch.warned_drop = true;
  }
}

void CommandGateNode::onHeartbeatCb(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  bool was_open = isGateOpen();
  last_heartbeat_time_ = get_clock()->now();
  heartbeat_ok_ = true;
  bool now_open = isGateOpen();
  if (!was_open && now_open) {
    onGateStateChange(false, true, openReason("heartbeat received"));
  }
}

void CommandGateNode::heartbeatCheckTimerCb()
{
  bool was_open = isGateOpen();
  if (require_heartbeat_) {
    double elapsed = (get_clock()->now() - last_heartbeat_time_).seconds();
    bool prev_ok = heartbeat_ok_;
    heartbeat_ok_ = (elapsed < heartbeat_timeout_);
    bool now_open = isGateOpen();
    if (was_open && !now_open) {
      onGateStateChange(true, false, "heartbeat timeout");
    } else if (!prev_ok && heartbeat_ok_ && !was_open && now_open) {
      onGateStateChange(false, true, openReason("heartbeat received"));
    }
  }
}

void CommandGateNode::onEnableTopicCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool was_open = isGateOpen();
  enabled_ = msg->data;
  bool now_open = isGateOpen();
  if (was_open != now_open) {
    std::string reason = now_open ? openReason("enabled via topic") : "disabled via topic";
    onGateStateChange(was_open, now_open, reason);
  }
}

void CommandGateNode::onSetEnabledCb(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  bool was_open = isGateOpen();
  enabled_ = request->data;
  bool now_open = isGateOpen();
  if (was_open != now_open) {
    std::string reason = now_open ? openReason("enabled via service") : "disabled via service";
    onGateStateChange(was_open, now_open, reason);
  }
  response->success = true;
  response->message = std::string("enabled set to ") + (enabled_ ? "true" : "false");
}

}  // namespace command_gate

RCLCPP_COMPONENTS_REGISTER_NODE(command_gate::CommandGateNode)
