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

#ifndef AD_STATUS_LAMP_MANAGER__AD_STATUS_LAMP_MANAGER_HPP_
#define AD_STATUS_LAMP_MANAGER__AD_STATUS_LAMP_MANAGER_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_external_api_msgs/msg/hazard_status_stamped.hpp>

namespace ad_status_lamp_manager
{

// Local constants to replace autoware_state_machine_msgs::msg::StateMachine
namespace ServiceLayerState
{
  constexpr uint16_t STATE_UNDEFINED = 0;
  constexpr uint16_t STATE_DURING_WAKEUP = 1;
  constexpr uint16_t STATE_DURING_CLOSE = 2;
  constexpr uint16_t STATE_CHECK_NODE_ALIVE = 3;
  constexpr uint16_t STATE_DURING_RECEIVE_ROUTE = 4;
  constexpr uint16_t STATE_EMERGENCY_STOP = 5;
  constexpr uint16_t STATE_OTHER = 0xFFFF;
}  // namespace ServiceLayerState

namespace ControlLayerState
{
  constexpr uint8_t MANUAL = 0;
  constexpr uint8_t AUTO = 1;
}  // namespace ControlLayerState

class AdStatusLampManager : public rclcpp::Node
{
public:
  explicit AdStatusLampManager(const rclcpp::NodeOptions & options);
  ~AdStatusLampManager();

  enum BlinkType
  {
    BLINK_FAST = 0,
    BLINK_SLOW
  };

  // 状態遷移条件の型定義
  using StateCondition = std::function<bool()>;
  struct StateTransition
  {
    StateCondition condition;
    uint16_t state;
  };

  // Publisher
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_ad_status_lamp_;

  // Subscriber
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr sub_initilization_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_routing_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::HazardStatusStamped>::SharedPtr sub_hazard_status_;

  #define BLINK_FAST_ON_DURATION (0.2)
  #define BLINK_FAST_OFF_DURATION (0.2)
  #define BLINK_FAST_IDLE_DURATION (1.5)
  #define BLINK_SLOW_ON_DURATION (1.0)
  #define BLINK_SLOW_OFF_DURATION (1.0)
  #define ACTIVE_POLARITY (false)

  std::array<double, 4> fast_blink_duration_table_ = {
    BLINK_FAST_IDLE_DURATION,
    BLINK_FAST_ON_DURATION,
    BLINK_FAST_OFF_DURATION,
    BLINK_FAST_ON_DURATION
  };

  std::array<double, 2> slow_blink_duration_table_ = {
    BLINK_SLOW_OFF_DURATION,
    BLINK_SLOW_ON_DURATION
  };

  rclcpp::TimerBase::SharedPtr blink_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  uint64_t blink_sequence_;
  int blink_type_;
  bool active_polarity_;
  uint16_t service_layer_state_;
  uint16_t pre_service_layer_state_;
  uint16_t control_layer_state_;
  uint16_t pre_control_layer_state_;
  uint16_t initilization_state_;
  uint16_t routing_state_;
  bool em_holding_;
  autoware_adapi_v1_msgs::msg::OperationModeState operation_state_;

  // 状態遷移テーブル（優先順位順）
  std::vector<StateTransition> state_transitions_;

  void publishLampState(const bool value);
  void lampManager(const uint16_t service_layer_state, const uint8_t control_layer_state);
  void startLampBlinkOperation(int blink_type);
  void lampBlinkOperationCallback(void);
  double getTimerDuration(void);
  void setPeriod(const double new_period);
  void callbackAutowareInitializationMessage(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  void callbackRoutingStateMessage(
    const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  void callbackOperationModeStateMessage(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);
  void callbackHazardStatusMessage(
    const tier4_external_api_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);
  void changeState(void);
  void initOnTimer(void);
  static std::string getServiceLayerStateName(uint16_t state);
  static std::string getControlLayerStateName(uint16_t state);
};

}  // namespace ad_status_lamp_manager
#endif  // AD_STATUS_LAMP_MANAGER__AD_STATUS_LAMP_MANAGER_HPP_
