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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <dio_ros_driver/msg/dio_port.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_system_msgs/msg/hazard_status_stamped.hpp>

#include "ad_status_lamp_manager/ad_status_lamp_manager.hpp"

using LocalizationInitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using HazardStatusStamped = autoware_system_msgs::msg::HazardStatusStamped;
using DIOPort = dio_ros_driver::msg::DIOPort;

class AdStatusLampManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<ad_status_lamp_manager::AdStatusLampManager>(
      rclcpp::NodeOptions());

    auto qos = rclcpp::QoS(1).reliable().transient_local();

    pub_initialization_state_ = node_->create_publisher<LocalizationInitializationState>(
      "/api/localization/initialization_state", qos);
    pub_route_state_ = node_->create_publisher<RouteState>("/api/routing/state", qos);
    pub_hazard_status_ = node_->create_publisher<HazardStatusStamped>(
      "/system/emergency/hazard_status", qos);
    pub_operation_mode_ = node_->create_publisher<OperationModeState>(
      "/api/operation_mode/state", qos);

    sub_ = node_->create_subscription<DIOPort>(
      "ad_status_lamp_out", qos,
      [this](DIOPort::SharedPtr msg) { received_messages_.push_back(*msg); });
  }

  void TearDown() override { rclcpp::shutdown(); }

  void spinUntilMessage()
  {
    auto start_time = std::chrono::steady_clock::now();
    while (received_messages_.empty() &&
           std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
      rclcpp::spin_some(node_);
    }
  }

  void spinSome()
  {
    for (int i = 0; i < 5; ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void publishInitializationState(uint16_t state)
  {
    LocalizationInitializationState msg;
    msg.state = state;
    pub_initialization_state_->publish(msg);
  }

  void publishRouteState(uint16_t state)
  {
    RouteState msg;
    msg.state = state;
    pub_route_state_->publish(msg);
  }

  void publishHazardStatus(bool emergency_holding)
  {
    HazardStatusStamped msg;
    msg.status.emergency_holding = emergency_holding;
    pub_hazard_status_->publish(msg);
  }

  void publishOperationMode(bool is_autoware_control, bool is_stop_mode_available,
                            bool is_local_mode_available)
  {
    OperationModeState msg;
    msg.is_autoware_control_enabled = is_autoware_control;
    msg.is_stop_mode_available = is_stop_mode_available;
    msg.is_local_mode_available = is_local_mode_available;
    pub_operation_mode_->publish(msg);
  }

  std::shared_ptr<ad_status_lamp_manager::AdStatusLampManager> node_;
  rclcpp::Publisher<LocalizationInitializationState>::SharedPtr pub_initialization_state_;
  rclcpp::Publisher<RouteState>::SharedPtr pub_route_state_;
  rclcpp::Publisher<HazardStatusStamped>::SharedPtr pub_hazard_status_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Subscription<DIOPort>::SharedPtr sub_;
  std::vector<DIOPort> received_messages_;
};

// Test: getServiceLayerStateName returns correct state names
TEST_F(AdStatusLampManagerTest, TestGetServiceLayerStateName)
{
  using namespace ad_status_lamp_manager;

  EXPECT_EQ(AdStatusLampManager::getServiceLayerStateName(ServiceLayerState::STATE_UNDEFINED),
            "STATE_UNDEFINED");
  EXPECT_EQ(AdStatusLampManager::getServiceLayerStateName(ServiceLayerState::STATE_DURING_WAKEUP),
            "STATE_DURING_WAKEUP");
  EXPECT_EQ(AdStatusLampManager::getServiceLayerStateName(ServiceLayerState::STATE_DURING_CLOSE),
            "STATE_DURING_CLOSE");
  EXPECT_EQ(AdStatusLampManager::getServiceLayerStateName(ServiceLayerState::STATE_CHECK_NODE_ALIVE),
            "STATE_CHECK_NODE_ALIVE");
  EXPECT_EQ(AdStatusLampManager::getServiceLayerStateName(ServiceLayerState::STATE_DURING_RECEIVE_ROUTE),
            "STATE_DURING_RECEIVE_ROUTE");
  EXPECT_EQ(AdStatusLampManager::getServiceLayerStateName(ServiceLayerState::STATE_EMERGENCY_STOP),
            "STATE_EMERGENCY_STOP");
  EXPECT_EQ(AdStatusLampManager::getServiceLayerStateName(ServiceLayerState::STATE_OTHER),
            "STATE_OTHER");
  EXPECT_EQ(AdStatusLampManager::getServiceLayerStateName(9999), "UNKNOWN");
}

// Test: getControlLayerStateName returns correct state names
TEST_F(AdStatusLampManagerTest, TestGetControlLayerStateName)
{
  using namespace ad_status_lamp_manager;

  EXPECT_EQ(AdStatusLampManager::getControlLayerStateName(ControlLayerState::MANUAL), "MANUAL");
  EXPECT_EQ(AdStatusLampManager::getControlLayerStateName(ControlLayerState::AUTO), "AUTO");
  EXPECT_EQ(AdStatusLampManager::getControlLayerStateName(99), "UNKNOWN");
}

// Test: Initial state should be STATE_UNDEFINED
TEST_F(AdStatusLampManagerTest, TestInitialStateIsUndefined)
{
  using namespace ad_status_lamp_manager;

  // Initial state should be STATE_UNDEFINED before any callbacks
  EXPECT_EQ(node_->service_layer_state_, ServiceLayerState::STATE_UNDEFINED);
}

// Test: After init timer fires, state should transition from STATE_UNDEFINED
TEST_F(AdStatusLampManagerTest, TestInitialStateTransitionAfterTimer)
{
  using namespace ad_status_lamp_manager;

  // Wait for init timer to fire (1 second timer)
  auto start_time = std::chrono::steady_clock::now();
  while (node_->service_layer_state_ == ServiceLayerState::STATE_UNDEFINED &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // After init timer fires, state should no longer be STATE_UNDEFINED
  // Note: Without any topic messages, the state transitions to STATE_OTHER
  // because no conditions in state_transitions_ are met
  EXPECT_NE(node_->service_layer_state_, ServiceLayerState::STATE_UNDEFINED);
}

// Test: Emergency holding should trigger STATE_EMERGENCY_STOP
TEST_F(AdStatusLampManagerTest, TestEmergencyStopState)
{
  using namespace ad_status_lamp_manager;

  spinSome();  // Let the node initialize

  publishHazardStatus(true);
  spinSome();

  EXPECT_EQ(node_->service_layer_state_, ServiceLayerState::STATE_EMERGENCY_STOP);
}

// Test: Route state UNSET should trigger STATE_DURING_RECEIVE_ROUTE
TEST_F(AdStatusLampManagerTest, TestRoutingStateUnset)
{
  using namespace ad_status_lamp_manager;

  spinSome();

  publishRouteState(RouteState::UNSET);
  spinSome();

  EXPECT_EQ(node_->service_layer_state_, ServiceLayerState::STATE_DURING_RECEIVE_ROUTE);
}

// Test: Route state CHANGING should trigger STATE_DURING_RECEIVE_ROUTE
TEST_F(AdStatusLampManagerTest, TestRoutingStateChanging)
{
  using namespace ad_status_lamp_manager;

  spinSome();

  publishRouteState(RouteState::CHANGING);
  spinSome();

  EXPECT_EQ(node_->service_layer_state_, ServiceLayerState::STATE_DURING_RECEIVE_ROUTE);
}

// Test: Route state ARRIVED should trigger STATE_DURING_RECEIVE_ROUTE
TEST_F(AdStatusLampManagerTest, TestRoutingStateArrived)
{
  using namespace ad_status_lamp_manager;

  spinSome();

  publishRouteState(RouteState::ARRIVED);
  spinSome();

  EXPECT_EQ(node_->service_layer_state_, ServiceLayerState::STATE_DURING_RECEIVE_ROUTE);
}

// Test: Initialization state INITIALIZING with stop_mode_available should trigger STATE_DURING_WAKEUP
TEST_F(AdStatusLampManagerTest, TestDuringWakeupState)
{
  using namespace ad_status_lamp_manager;

  spinSome();

  // Set routing state to SET (not UNSET/CHANGING/ARRIVED)
  publishRouteState(RouteState::SET);
  spinSome();

  // Set operation mode with stop_mode_available = true
  publishOperationMode(false, true, false);
  spinSome();

  // Set initialization state to INITIALIZING
  publishInitializationState(LocalizationInitializationState::INITIALIZING);
  spinSome();

  EXPECT_EQ(node_->service_layer_state_, ServiceLayerState::STATE_DURING_WAKEUP);
}

// Test: Control layer state changes based on is_autoware_control_enabled
TEST_F(AdStatusLampManagerTest, TestControlLayerStateManual)
{
  using namespace ad_status_lamp_manager;

  spinSome();

  // is_autoware_control_enabled = false -> MANUAL
  publishOperationMode(false, false, false);
  spinSome();

  EXPECT_EQ(node_->control_layer_state_, ControlLayerState::MANUAL);
}

// Test: Control layer state AUTO when is_autoware_control_enabled = true
TEST_F(AdStatusLampManagerTest, TestControlLayerStateAuto)
{
  using namespace ad_status_lamp_manager;

  spinSome();

  publishOperationMode(true, false, false);
  spinSome();

  EXPECT_EQ(node_->control_layer_state_, ControlLayerState::AUTO);
}

// Test: Emergency stop has higher priority than routing state
TEST_F(AdStatusLampManagerTest, TestEmergencyStopPriority)
{
  using namespace ad_status_lamp_manager;

  spinSome();

  // Set routing state to UNSET
  publishRouteState(RouteState::UNSET);
  spinSome();

  // Emergency holding should take priority
  publishHazardStatus(true);
  spinSome();

  EXPECT_EQ(node_->service_layer_state_, ServiceLayerState::STATE_EMERGENCY_STOP);
}

// Test: Lamp output is published
TEST_F(AdStatusLampManagerTest, TestLampOutputPublished)
{
  received_messages_.clear();

  spinUntilMessage();

  ASSERT_FALSE(received_messages_.empty());
}
