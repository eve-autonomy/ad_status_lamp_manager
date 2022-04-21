// Copyright 2020 eve autonomy inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#include <memory>
#include <utility>
#include <array>
#include "ad_status_lamp_manager/ad_status_lamp_manager.hpp"

namespace ad_status_lamp_manager
{

AdStatusLampManager::AdStatusLampManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("ad_status_lamp_manager", options)
{
  sub_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateMachine>(
    "autoware_state_machine/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdStatusLampManager::callbackStateMessage, this, std::placeholders::_1)
  );
  pub_ad_status_lamp_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "ad_status_lamp_out",
    rclcpp::QoS{3}.transient_local());

  active_polarity_ = ACTIVE_POLARITY;

  // Timer
  std::chrono::milliseconds timer_period_msec;
  timer_period_msec = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0));

  auto timer_callback = std::bind(&AdStatusLampManager::lampBlinkOperationCallback, this);
  blink_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), timer_period_msec, std::move(timer_callback),
    this->get_node_base_interface()->get_context()
  );
  this->get_node_timers_interface()->add_timer(blink_timer_, nullptr);
  blink_timer_->cancel();

  publishLampState(false);
}

AdStatusLampManager::~AdStatusLampManager()
{
  publishLampState(false);
  blink_timer_->cancel();
}

void AdStatusLampManager::callbackStateMessage(
  const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdStatusLampManager::callbackStateMessage]service_layer_state: %u, control_layer_state: %u",
    msg->service_layer_state,
    msg->control_layer_state);

  lampManager(msg->service_layer_state, msg->control_layer_state);
}

void AdStatusLampManager::publishLampState(const bool value)
{
  dio_ros_driver::msg::DIOPort msg;
  msg.value = active_polarity_ ? value : !value;
  pub_ad_status_lamp_->publish(msg);
}

void AdStatusLampManager::lampManager(
  const uint16_t service_layer_state, const uint8_t control_layer_state)
{
  switch (service_layer_state) {
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_CLOSE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE:
      // slow blink
      startLampBlinkOperation(BLINK_SLOW);
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE:
      // fast blink
      startLampBlinkOperation(BLINK_FAST);
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP:
      // lamp on
      blink_timer_->cancel();
      publishLampState(true);
      break;

    default:
      if (control_layer_state == autoware_state_machine_msgs::msg::StateMachine::MANUAL) {
        // fast blink
        startLampBlinkOperation(BLINK_FAST);
      } else {
        // lamp on
        blink_timer_->cancel();
        publishLampState(true);
      }
      break;
  }
}

double AdStatusLampManager::getTimerDuration(void)
{
  if (blink_type_ == BLINK_FAST) {
    if (fast_blink_duration_table_.size() <= blink_sequence_) {
      blink_sequence_ = 0;
    }
    return fast_blink_duration_table_.at(blink_sequence_);
  } else {
    if (slow_blink_duration_table_.size() <= blink_sequence_) {
      blink_sequence_ = 0;
    }
    return slow_blink_duration_table_.at(blink_sequence_);
  }
}

void AdStatusLampManager::startLampBlinkOperation(int blink_type)
{
  blink_timer_->cancel();

  blink_sequence_ = 0;
  blink_type_ = blink_type;
  double duration = getTimerDuration();

  publishLampState(false);

  setPeriod(duration);
}

void AdStatusLampManager::lampBlinkOperationCallback(void)
{
  blink_timer_->cancel();

  blink_sequence_++;
  double duration = getTimerDuration();

  // odd sequence -> true : even sequence -> false
  publishLampState(blink_sequence_ % 2 ? true : false);

  setPeriod(duration);
}

void AdStatusLampManager::setPeriod(const double new_period)
{
  int64_t old_period = 0;
  std::chrono::nanoseconds period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(new_period));

  int64_t timer_period = period.count();

  rcl_ret_t ret = rcl_timer_exchange_period(
    blink_timer_->get_timer_handle().get(), timer_period, &old_period);
  if (ret != RCL_RET_OK) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1.0,
      "Couldn't exchange_period");
  }
  blink_timer_->reset();
}

}  // namespace ad_status_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ad_status_lamp_manager::AdStatusLampManager)
