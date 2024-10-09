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

using unconstrained_mpc_controller::types::state_vector_t;
using unconstrained_mpc_controller::types::output_vector_t;
using unconstrained_mpc_controller::types::control_vector_t;

class TestableUnconstrainedMpcController
  : public unconstrained_mpc_controller::UnconstrainedMpcController
{
public:
  /**
   * @brief Block waiting for the futures referencesc to be published or timeout
   *
   * Required that the executor is not spinned elsewhere between the message publication and the
   * call to this function
   */
  void wait_for_future_references(
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
    this->fake_node = std::make_shared<rclcpp::Node>("fake_node");

    this->future_refs_pub = fake_node->create_publisher<std_msgs::msg::Float64MultiArray>(
      future_refs_topic_name, 10);

    rclcpp::get_logger(controller_name).set_level(rclcpp::Logger::Level::Debug);
    this->config_path = ament_index_cpp::get_package_prefix("unconstrained_mpc_controller") +
                  "/test/config/test_unconstrained_mpc_controller_params.yaml";
  }

  void InitController()
  {
    auto node_options = this->controller->define_custom_node_options();
    node_options.arguments({"--ros-args", "--params-file", config_path});
    ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
      controller_interface::return_type::OK);
  }

  void ConfigureController()
  {
    ASSERT_EQ(
      controller->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  void ActivateController()
  {
    ASSERT_EQ(
      controller->on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  void AssignResourcesToController()
  {
    const std::vector<std::string> control_input_ifaces{
      "cmd_interface/u1", "cmd_interface/u2", "cmd_interface/u3", "cmd_interface/u4"};
    const std::vector<std::string> state_ifaces{
      "state_interface/x1", "state_interface/x2", "state_interface/x3", "state_interface/x4"};
    const std::vector<std::string> output_ifaces{
      "output_interface/y1", "output_interface/y2", "output_interface/y3", "output_interface/y4"};

    command_interfaces.reserve(control_input_ifaces.size());
    state_interfaces.reserve(state_ifaces.size() + output_ifaces.size());

    for (size_t i = 0; i < control_input_ifaces.size(); i++) {
      auto command_interface = std::make_unique<hardware_interface::CommandInterface>(
        "cmd_interface", "u" + std::to_string(i+1), &plant_control_input_values[i]);
      command_interfaces.emplace_back(std::move(command_interface));
    }

    for (size_t i = 0; i < state_ifaces.size(); i++) {
      auto state_interface = std::make_unique<hardware_interface::StateInterface>(
        "state_interface", "x" + std::to_string(i+1), &plant_state_values[i]);
      state_interfaces.emplace_back(std::move(state_interface));
    }

    for (size_t i = 0; i < output_ifaces.size(); i++) {
      auto state_interface = std::make_unique<hardware_interface::StateInterface>(
        "output_interface", "y" + std::to_string(i+1), &plant_output_values[i]);
      state_interfaces.emplace_back(std::move(state_interface));
    }

    std::vector<hardware_interface::LoanedCommandInterface> tmp_loaned_command_interfaces;
    for (auto & command_interface : command_interfaces) {
      tmp_loaned_command_interfaces.emplace_back(*command_interface);
    }

    std::vector<hardware_interface::LoanedStateInterface> tmp_loaned_state_interfaces;
    for (auto & state_interface : state_interfaces) {
      tmp_loaned_state_interfaces.emplace_back(*state_interface);
    }

    this->controller->assign_interfaces(
      std::move(tmp_loaned_command_interfaces),
      std::move(tmp_loaned_state_interfaces));
  }

  /// \brief wait for the subscriber and publisher to completely setup
  void WaitForSetup()
  {
    constexpr std::chrono::seconds TIMEOUT{2};
    auto clock = fake_node->get_clock();
    auto start = clock->now();
    while (future_refs_pub->get_subscription_count() <= 0) {
      if ((clock->now() - start) > TIMEOUT) {
        FAIL();
      }
      rclcpp::spin_some(fake_node);
    }
  }

  void PublishFutureRefs(const std::vector<double> & future_refs)
  {
    int wait_count = 0;
    auto topic = future_refs_pub->get_topic_name();
    while (fake_node->count_subscribers(topic) == 0) {
      if (wait_count >= 5) {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = future_refs;
    future_refs_pub->publish(msg);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr fake_node;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr future_refs_pub;

  const std::string controller_name{"unconstrained_mpc_controller"};
  const std::string future_refs_topic_name{controller_name + "/future_refs"};
  std::unique_ptr<TestableUnconstrainedMpcController> controller;
  std::string config_path;

  std::vector<std::unique_ptr<hardware_interface::CommandInterface>> command_interfaces;
  std::vector<std::unique_ptr<hardware_interface::StateInterface>> state_interfaces;

  state_vector_t plant_state_values{state_vector_t::Zero(4)};
  output_vector_t plant_output_values{output_vector_t::Zero(4)};
  control_vector_t plant_control_input_values{control_vector_t::Zero(4)};
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
    controller_interface::CallbackReturn::FAILURE);
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
    controller_interface::CallbackReturn::FAILURE);
}

TEST_F(
  FixtureforUnconstrainedMpcController,
  Configuration_fails_with_insufficient_required_plant_control_input_ifaces_names)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  node_options.append_parameter_override(
    "required_hw_ifaces.plant.control_input", std::vector<std::string>{"cmd_interface/u1"});

  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

TEST_F(
  FixtureforUnconstrainedMpcController,
  Configuration_fails_with_insufficient_required_plant_state_ifaces_names)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  node_options.append_parameter_override(
    "required_hw_ifaces.plant.control_input", std::vector<std::string>{"state_interface/x1"});

  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

TEST_F(
  FixtureforUnconstrainedMpcController,
  Configuration_fails_with_invalid_required_plant_output_ifaces_names)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  node_options.append_parameter_override(
    "required_hw_ifaces.plant.output", std::vector<std::string>{"output_interface/y1"});

  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

TEST_F(FixtureforUnconstrainedMpcController, Claim_valid_command_interfaces)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  auto cmd_ifaces = this->controller->command_interface_configuration();
  // From the test_unconstrained_mpc_controller_params.yaml
  ASSERT_THAT(cmd_ifaces.names, ::testing::SizeIs(4));
  EXPECT_STREQ(cmd_ifaces.names[0].c_str(), "cmd_interface/u1");
  EXPECT_STREQ(cmd_ifaces.names[1].c_str(), "cmd_interface/u2");
  EXPECT_STREQ(cmd_ifaces.names[2].c_str(), "cmd_interface/u3");
  EXPECT_STREQ(cmd_ifaces.names[3].c_str(), "cmd_interface/u4");
}

TEST_F(FixtureforUnconstrainedMpcController, Claim_valid_state_interfaces)
{
  auto node_options = this->controller->define_custom_node_options();
  node_options.arguments({"--ros-args", "--params-file", config_path});
  ASSERT_EQ(this->controller->init(controller_name, "", 1, "", node_options),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    this->controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // The MPC controller also need ros2_control state interfaces that are used to read the plant
  // output and the states that are used by the MPC control law to compute the control signal.
  const std::vector<std::string> expected_state_ifaces = {
    "state_interface/x1",
    "state_interface/x2",
    "state_interface/x3",
    "state_interface/x4",
    "output_interface/y1",
    "output_interface/y2",
    "output_interface/y3",
    "output_interface/y4",
  };

  auto state_ifaces = this->controller->state_interface_configuration();
  // From the test_unconstrained_mpc_controller_params.yaml
  EXPECT_THAT(state_ifaces.names, ::testing::SizeIs(8));
  EXPECT_THAT(state_ifaces.names, ::testing::UnorderedElementsAreArray(expected_state_ifaces));
}

TEST_F(FixtureforUnconstrainedMpcController, Update_success)
{
  this->InitController();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->get_node()->get_node_base_interface());

  auto state = this->controller->get_node()->configure();
  ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  this->AssignResourcesToController();
  state = this->controller->get_node()->activate();
  ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  this->WaitForSetup();

  std::vector<double> future_refs_values;

  for (size_t i = 0; i < 100; i++) {
    future_refs_values.push_back(static_cast<double>(1));
  }

  this->PublishFutureRefs(future_refs_values);

  auto time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  auto period = rclcpp::Duration::from_seconds(0.01);

  this->controller->wait_for_future_references(executor);
  auto update_ret = this->controller->update(time, period);
  ASSERT_EQ(update_ret, controller_interface::return_type::OK);

  EXPECT_THAT(plant_control_input_values, ::testing::Each(::testing::DoubleEq(0.0)));

  update_ret = this->controller->update(time, period);
  ASSERT_EQ(update_ret, controller_interface::return_type::OK);

  EXPECT_THAT(plant_control_input_values, ::testing::Each(::testing::DoubleEq(6.0)));
}
