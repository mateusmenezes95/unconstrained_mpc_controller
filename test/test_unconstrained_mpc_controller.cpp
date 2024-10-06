// Copyright (c) 2024 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gmock/gmock.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "unconstrained_mpc_controller_params.hpp"  // NOLINT
#include "unconstrained_mpc_controller/types/mpc_definitions.hpp"
#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"
#include "unconstrained_mpc_controller/types/types.hpp"


#include "unconstrained_mpc_controller/unconstrained_mpc_controller.hpp"

class TestableUnconstrainedMpcController
  : public unconstrained_mpc_controller::UnconstrainedMpcController
{
public:
  /**
   * @brief Block waiting for the generalized control forces to be published or timeout Requires
   *
   * that the executor is not spinned elsewhere between the message publication and the call to this
   * function
   */
  void wait_for_generalized_control_forces(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(500))
  {
    auto until = this->get_node()->get_clock()->now() + timeout;
    while (this->get_node()->get_clock()->now() < until) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }
};

struct FixtureforUnconstrainedMpcController : public ::testing::Test
{
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp() override
  {
    this->controller = std::make_unique<TestableUnconstrainedMpcController>();

    rclcpp::get_logger(controller_name).set_level(rclcpp::Logger::Level::Debug);
    this->config_path = ament_index_cpp::get_package_prefix("unconstrained_mpc_controller") +
                  "/test/config/test_unconstrained_mpc_controller_params.yaml";
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  const std::string controller_name{"unconstrained_mpc_controller"};
  std::unique_ptr<TestableUnconstrainedMpcController> controller;
  std::string config_path;
};

TEST_F(FixtureforUnconstrainedMpcController, Initialization_fails_without_parameters)
{
  ASSERT_EQ(
    this->controller->init(controller_name, "", 1, "", controller->define_custom_node_options()),
    controller_interface::return_type::ERROR);
}

TEST_F(
  FixtureforUnconstrainedMpcController,
  Configuration_fails_when_the_control_horizon_is_greater_than_the_prediction_horizon)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  node_options.append_parameter_override("prediction_horizon", 1);
  node_options.append_parameter_override("control_horizon", 3);

  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(
  FixtureforUnconstrainedMpcController,
  Configuration_fails_with_invalid_mpc_gains_vector_size)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  node_options.append_parameter_override("prediction_horizon", 2);
  node_options.append_parameter_override("control_horizon", 1);

  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(
  FixtureforUnconstrainedMpcController,
  Configuration_fails_with_insufficient_required_cmd_ifaces_names)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  node_options.append_parameter_override(
    "required_cmd_ifaces_names", std::vector<std::string>{"cmd_interface/u1"});

  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

TEST_F(
  FixtureforUnconstrainedMpcController,
  Configuration_fails_with_insufficient_required_state_ifaces_names)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  node_options.append_parameter_override(
    "required_state_ifaces_names", std::vector<std::string>{"state_interface/x1"});

  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}