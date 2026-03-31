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

  // HazardStatus for EM Holding (/api/external/get/hazard_status)
  sub_hazard_status_ = this->create_subscription<tier4_external_api_msgs::msg::HazardStatusStamped>(
    "/api/external/get/hazard_status",
    rclcpp::QoS{1},
    std::bind(&AdStatusLampManager::callbackHazardStatusMessage, this, std::placeholders::_1)
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
  service_layer_state_ = ServiceLayerState::STATE_UNDEFINED;
  pre_service_layer_state_ = ServiceLayerState::STATE_UNDEFINED;
  control_layer_state_ = ControlLayerState::MANUAL;
  pre_control_layer_state_ = ControlLayerState::MANUAL;
  initilization_state_ = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNKNOWN;
  routing_state_ = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
  em_holding_ = false;
  operation_state_.is_autoware_control_enabled = false;
  operation_state_.is_in_transition = false;
  operation_state_.is_stop_mode_available = false;
  operation_state_.is_autonomous_mode_available = false;
  operation_state_.is_local_mode_available = false;
  operation_state_.is_remote_mode_available = false;

  // 状態遷移テーブルの初期化（優先順位順に評価される）
  state_transitions_ = {
    {
      [this]() { return service_layer_state_ == ServiceLayerState::STATE_UNDEFINED; },
      ServiceLayerState::STATE_CHECK_NODE_ALIVE
    },
    {
      [this]() { return em_holding_; },
      ServiceLayerState::STATE_EMERGENCY_STOP
    },
    {
      [this]() {
        return routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::UNSET ||
               routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::CHANGING ||
               routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
      },
      ServiceLayerState::STATE_DURING_RECEIVE_ROUTE
    },
    {
      [this]() {
        return initilization_state_ == autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING &&
               (operation_state_.is_stop_mode_available || operation_state_.is_local_mode_available);
      },
      ServiceLayerState::STATE_DURING_WAKEUP
    }
  };

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
  initilization_state_ = msg->state;

  changeState();
  lampManager(service_layer_state_, control_layer_state_);
}

void AdStatusLampManager::callbackRoutingStateMessage(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  routing_state_ = msg->state;

  changeState();
  lampManager(service_layer_state_, control_layer_state_);
}

void AdStatusLampManager::callbackHazardStatusMessage(
  const tier4_external_api_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  em_holding_ = msg->status.emergency_holding;

  changeState();
  lampManager(service_layer_state_, control_layer_state_);
}

void AdStatusLampManager::callbackOperationModeStateMessage(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  operation_state_ = *msg;

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
    case ServiceLayerState::STATE_DURING_WAKEUP:
    case ServiceLayerState::STATE_DURING_CLOSE:
    case ServiceLayerState::STATE_CHECK_NODE_ALIVE:
      // slow blink
      startLampBlinkOperation(BLINK_SLOW);
      break;

    case ServiceLayerState::STATE_DURING_RECEIVE_ROUTE:
      // fast blink
      startLampBlinkOperation(BLINK_FAST);
      break;

    case ServiceLayerState::STATE_EMERGENCY_STOP:
      // lamp on
      blink_timer_->cancel();
      publishLampState(true);
      break;

    default:
      if (control_layer_state == ControlLayerState::MANUAL) {
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
  // 状態遷移テーブルを順番に評価し、最初にマッチした状態を設定
  service_layer_state_ = ServiceLayerState::STATE_OTHER;
  for (const auto & transition : state_transitions_) {
    if (transition.condition()) {
      service_layer_state_ = transition.state;
      break;
    }
  }

  // control_layer_state の更新
  control_layer_state_ = operation_state_.is_autoware_control_enabled
    ? ControlLayerState::AUTO
    : ControlLayerState::MANUAL;
}

std::string AdStatusLampManager::getServiceLayerStateName(uint16_t state)
{
  switch (state) {
    case ServiceLayerState::STATE_UNDEFINED:
      return "STATE_UNDEFINED";
    case ServiceLayerState::STATE_DURING_WAKEUP:
      return "STATE_DURING_WAKEUP";
    case ServiceLayerState::STATE_DURING_CLOSE:
      return "STATE_DURING_CLOSE";
    case ServiceLayerState::STATE_CHECK_NODE_ALIVE:
      return "STATE_CHECK_NODE_ALIVE";
    case ServiceLayerState::STATE_DURING_RECEIVE_ROUTE:
      return "STATE_DURING_RECEIVE_ROUTE";
    case ServiceLayerState::STATE_EMERGENCY_STOP:
      return "STATE_EMERGENCY_STOP";
    case ServiceLayerState::STATE_OTHER:
      return "STATE_OTHER";
    default:
      return "UNKNOWN";
  }
}

std::string AdStatusLampManager::getControlLayerStateName(uint16_t state)
{
  switch (state) {
    case ControlLayerState::MANUAL:
      return "MANUAL";
    case ControlLayerState::AUTO:
      return "AUTO";
    default:
      return "UNKNOWN";
  }
}
}  // namespace ad_status_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ad_status_lamp_manager::AdStatusLampManager)
