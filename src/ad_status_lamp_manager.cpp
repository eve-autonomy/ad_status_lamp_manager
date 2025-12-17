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
  // subscriber
  // autoware_state
  sub_initilization_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
    "/api/localization/initialization_state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdStatusLampManager::callbackAutowareInitializationMessage, this, std::placeholders::_1)
  );

  // routing wait state
  sub_routing_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdStatusLampManager::callbackRoutingStateMessage, this, std::placeholders::_1)
  );

  // ad_sound_manager sound done state
  sub_sound_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateSoundDone>(
    "/autoware_state_machine/state_sound_done",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdStatusLampManager::callbackSoundDoneMessage, this, std::placeholders::_1)
  );

  // daignostics struct for EM Holding
  sub_daignostics_struct_ = this->create_subscription<autoware_adapi_v1_msgs::msg::DiagGraphStruct>(
    "/api/system/diagnostics/struct",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdStatusLampManager::callbackDaignosticsStructMessage, this, std::placeholders::_1)
  );

  // daignostics status for EM Holding
  sub_daignostics_status_ = this->create_subscription<autoware_adapi_v1_msgs::msg::DiagGraphStatus>(
    "/api/system/diagnostics/status",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdStatusLampManager::callbackDaignosticsStateMessage, this, std::placeholders::_1)
  );

  // OperationModeState
  sub_operation_mode_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdStatusLampManager::callbackOperationModeStateMessage, this, std::placeholders::_1)
  );

  // publisher
  // lamp state
  pub_ad_status_lamp_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "ad_status_lamp_out",
    rclcpp::QoS{3}.transient_local());

  // Set Initial Value
  active_polarity_ = ACTIVE_POLARITY;
  em_holding_indices_ = std::nullopt;
  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  pre_service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  pre_control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  initilization_state_ = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNKNOWN;
  routing_state_ = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
  sound_param_.state = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  sound_param_.done = false;
  em_holding_ = false;
  operation_state_.is_autoware_control_enabled = false;
  operation_state_.is_in_transition = false;
  operation_state_.is_stop_mode_available = false;
  operation_state_.is_autonomous_mode_available = false;
  operation_state_.is_local_mode_available = false;
  operation_state_.is_remote_mode_available = false;

  // Timer
  // Lamp on/off
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

  // initailize state
  auto init_timer_callback = std::bind(&AdStatusLampManager::initOnTimer, this);
  init_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(init_timer_callback)>>(
    this->get_clock(), timer_period_msec, std::move(init_timer_callback),
    this->get_node_base_interface()->get_context()
  );
  this->get_node_timers_interface()->add_timer(init_timer_, nullptr);

  publishLampState(false);
}

AdStatusLampManager::~AdStatusLampManager()
{
  publishLampState(false);
  blink_timer_->cancel();
}

void AdStatusLampManager::callbackAutowareInitializationMessage(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdStatusLampManager::callbackAutowareInitializationMessage]autoware_state: %u",
    msg->state);

  initilization_state_ = msg->state;

  changeState();
  lampManager(service_layer_state_, control_layer_state_);
}

void AdStatusLampManager::callbackRoutingStateMessage(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdStatusLampManager::callbackRoutingStateMessage]routing_state: %u",
    msg->state);

  routing_state_ = msg->state;

  changeState();
  lampManager(service_layer_state_, control_layer_state_);
}

void AdStatusLampManager::callbackSoundDoneMessage(
  const autoware_state_machine_msgs::msg::StateSoundDone::ConstSharedPtr msg_ptr)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdStatusLampManager::callbackSoundDoneMessage]Sound Done %d, %d ",
      msg_ptr->state, msg_ptr->done);

  sound_param_.state = msg_ptr->state;
  sound_param_.done = msg_ptr->done;

  changeState();
  lampManager(service_layer_state_, control_layer_state_);
}

void AdStatusLampManager::callbackDaignosticsStructMessage(
  const autoware_adapi_v1_msgs::msg::DiagGraphStruct::ConstSharedPtr msg)
{
  auto nodes = msg->nodes;

  for (uint16_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].path == "/autoware/modes/autonomous") {
      em_holding_indices_ = i;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1.0,
        "[AdStatusLampManager::callbackDaignosticsStructMessage]daignostics_graph /autoware/modes/autonomous index: %u", i);
      break;
    }
  }
}

void AdStatusLampManager::callbackDaignosticsStateMessage(
  const autoware_adapi_v1_msgs::msg::DiagGraphStatus::ConstSharedPtr msg)
{
  auto nodes = msg->nodes;
  if (em_holding_indices_ != std::nullopt) {
    // TODO:Ph3にて、levelをlatch_levelに変更
    if (nodes[em_holding_indices_.value()].level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      em_holding_ = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1.0,
        "[AdStatusLampManager::callbackDaignosticsStateMessage]/autoware/modes/autonomous latch_level: %u",
        nodes[em_holding_indices_.value()].level);// TODO:Ph3にて、levelをlatch_levelに変更
    } else {
      em_holding_ = false;
    }

    changeState();
    lampManager(service_layer_state_, control_layer_state_);
  }
}

void AdStatusLampManager::callbackOperationModeStateMessage(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  operation_state_ = *msg;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdStatusLampManager::callbackOperationModeStateMessage]operation mode: %u",
      msg->mode);

  changeState();
  lampManager(service_layer_state_, control_layer_state_);
}

void AdStatusLampManager::initOnTimer(void)
{
  init_timer_->cancel();
  changeState();
  lampManager(service_layer_state_, control_layer_state_);
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
  if (pre_service_layer_state_ == service_layer_state &&
      pre_control_layer_state_ == control_layer_state) {
    return;
  }

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

  pre_service_layer_state_ = service_layer_state;
  pre_control_layer_state_ = control_layer_state;
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

void AdStatusLampManager::changeState(void)
{
  if ((service_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED)){
    // STATE_CHECK_NODE_ALIVE
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE;
  } else if (em_holding_ == true) {
    // STATE_EMERGENCY_STOP
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
  } else if ((routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::UNSET) ||
             (routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::CHANGING) ||
             (routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED)) {
    // STATE_DURING_RECEIVE_ROUTE
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
  } else if ((initilization_state_ == autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING)
          && (((operation_state_.is_stop_mode_available == true)
            || (operation_state_.is_local_mode_available == true))
            || ((sound_param_.state == autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE)
              && (sound_param_.done == true)))) {
    // STATE_DURING_WAKEUP
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
  } else {
    // 上記以外
    service_layer_state_ = 0xFFFF;
  }

  if ((operation_state_.is_stop_mode_available == true) ||
      (operation_state_.is_local_mode_available == true)) {
    control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  } else {
    control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::AUTO;
  }
}
}  // namespace ad_status_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ad_status_lamp_manager::AdStatusLampManager)
