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

#include "rclcpp/rclcpp.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"

namespace ad_status_lamp_manager
{
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

  // Publisher
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_ad_status_lamp_;

  // Subscriber
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateMachine>::SharedPtr sub_state_;

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
  uint64_t blink_sequence_;
  int blink_type_;
  bool active_polarity_;

  void callbackStateMessage(
    const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg);
  void publishLampState(const bool value);
  void lampManager(const uint16_t service_layer_state, const uint8_t control_layer_state);
  void startLampBlinkOperation(int blink_type);
  void lampBlinkOperationCallback(void);
  double getTimerDuration(void);
  void setPeriod(const double new_period);
};

}  // namespace ad_status_lamp_manager
#endif  // AD_STATUS_LAMP_MANAGER__AD_STATUS_LAMP_MANAGER_HPP_
