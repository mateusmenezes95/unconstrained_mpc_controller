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

#include "unconstrained_mpc_controller/unconstrained_mpc_controller.hpp"

#include <fmt/format.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "unconstrained_mpc_controller_params.hpp"  // NOLINT
#include "unconstrained_mpc_controller/mpc_matrix_converter.hpp"
#include "unconstrained_mpc_controller/mpc_vectors_manipulator.hpp"
#include "unconstrained_mpc_controller/types/mpc_definitions.hpp"
#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"
#include "unconstrained_mpc_controller/types/types.hpp"

namespace unconstrained_mpc_controller
{

controller_interface::CallbackReturn UnconstrainedMpcController::on_init()
{
  this->node_ = this->get_node();
  RCLCPP_INFO(node_->get_logger(), "on_init");

  try {
    param_listener_ = std::make_shared<ParamListener>(this->get_node());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->node_->get_logger(), "UnconstrainedMpcController initialized");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_configure called. Previous state was '%s'",
    previous_state.label().c_str());

  RCLCPP_INFO(this->node_->get_logger(), "Configuring unconstrained_mpc_controller");

  try {
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  mpc_params_.state_size = params_.state_size;
  mpc_params_.control_size = params_.control_size;
  mpc_params_.output_size = params_.output_size;
  mpc_params_.prediction_horizon = params_.prediction_horizon;
  mpc_params_.control_horizon = params_.control_horizon;

  try {
    mpc_matrix_converter_ = std::make_unique<MpcMatrixConverter>(mpc_params_);
    mpc_vectors_manipulator_ = std::make_unique<MpcVectorsManipulator>(mpc_params_);
    kmpc_gain_ = mpc_matrix_converter_->getKmpcGain(params_.kmpc_gain);
    ky_gain_ = mpc_matrix_converter_->getKyGain(params_.ky_gain);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  this->resetPlantVectors();

  auto check_interfaces = [this](const auto & iface_names, size_t required_size,
    const std::string & iface_type) {
      if (iface_names.size() < required_size) {
        throw std::runtime_error(fmt::format(
          "Not enough {} interfaces names provided. Required at least {}, provided {}",
          iface_type, required_size, iface_names.size()));
      }
    };

  try {
    check_interfaces(
      params_.required_hw_ifaces.plant.control_input, params_.control_size, "plant control input");
    check_interfaces(
      params_.required_hw_ifaces.plant.state, params_.state_size, "plant state");
    check_interfaces(
      params_.required_hw_ifaces.plant.output, params_.output_size, "plant output");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  auto future_refs_callback = [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      RCLCPP_DEBUG(this->node_->get_logger(), "Received %zd future references", msg->data.size());
      if (!subscriber_is_active_) {
        RCLCPP_WARN(
        this->node_->get_logger(),
        "Received future references but subscriber is not active. "
        "Data will be ignored.");
        return;
      }
      future_refs_rt_buffer_.writeFromNonRT(msg);
      future_refs_rcvd_ = true;
    };

  try {
    future_refs_sub_ = this->get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/future_refs", rclcpp::SystemDefaultsQoS(), future_refs_callback);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  std_msgs::msg::Float64MultiArray::SharedPtr initial_generalized_ctrl_forces =
    std::make_shared<std_msgs::msg::Float64MultiArray>();
  initial_generalized_ctrl_forces->data.resize(100, 0.0);
  future_refs_rt_buffer_.writeFromNonRT(initial_generalized_ctrl_forces);

  future_refs_.clear();
  // Start with a vector of zeros. TODO(mmeneses): Allows to pass initial future refs as parameter
  future_refs_.resize(params_.prediction_horizon*params_.state_size, 0.0);
  current_desired_robot_vel.resize(params_.state_size, 0.0);

  control_input_rt_pub_ptr_.create(node_, "~/control_input");
  desired_robot_vel_rt_pub_ptr_.create(node_, "~/desired_robot_vel");
  robot_vel_rt_pub_ptr_.create(node_, "~/robot_vel");
  period_rt_pub_.create(node_, "~/control_loop_period");

  RCLCPP_INFO(this->node_->get_logger(), "Configuration of unconstrained mpc controller succeeded");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_cleanup called. Previous state was '%s'",
    previous_state.label().c_str());

  RCLCPP_INFO(this->node_->get_logger(), "Cleaning up unconstrained mpc controller");

  // It is not needed reset the plant vectors here since the controller is being cleaned up
  // and will be reconfigured in the next state transition.
  subscriber_is_active_ = false;
  future_refs_rcvd_ = false;
  future_refs_.clear();

  mpc_matrix_converter_.reset();
  mpc_vectors_manipulator_.reset();

  RCLCPP_INFO(this->node_->get_logger(), "unconstrained mpc controller cleaned up");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_activate called. Previous state was '%s'",
    previous_state.label().c_str());

  RCLCPP_INFO(this->node_->get_logger(), "Activating unconstrained mpc controller");

  subscriber_is_active_ = true;

  auto create_iface_to_index_map = [](const auto & iface_names) {
      std::unordered_map<std::string, size_t> iface_to_index_map;
      for (size_t i = 0; i < iface_names.size(); ++i) {
        iface_to_index_map[iface_names[i]] = i;
      }
      return iface_to_index_map;
    };

  try {
    eigen_hw_ifaces_bridge_.plant_control_input = std::make_unique<EigenVectorCmdInterfaceBridge>(
      this->command_interfaces_,
      create_iface_to_index_map(params_.required_hw_ifaces.plant.control_input));
    eigen_hw_ifaces_bridge_.plant_state = std::make_unique<StateInterfaceEigenVectorBridge>(
      this->state_interfaces_,
      create_iface_to_index_map(params_.required_hw_ifaces.plant.state));
    eigen_hw_ifaces_bridge_.plant_output = std::make_unique<StateInterfaceEigenVectorBridge>(
      this->state_interfaces_,
      create_iface_to_index_map(params_.required_hw_ifaces.plant.output));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->node_->get_logger(), "unconstrained mpc controller activated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_deactivate called. Previous state was '%s'",
    previous_state.label().c_str());

  RCLCPP_INFO(this->node_->get_logger(), "Deactivating unconstrained mpc controller");

  this->resetPlantVectors();

  subscriber_is_active_ = false;
  future_refs_rcvd_ = false;
  future_refs_.clear();

  eigen_hw_ifaces_bridge_.plant_control_input.reset();
  eigen_hw_ifaces_bridge_.plant_state.reset();
  eigen_hw_ifaces_bridge_.plant_output.reset();

  RCLCPP_INFO(this->node_->get_logger(), "unconstrained mpc controller deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_shutdown called. Previous state was '%s'",
    previous_state.label().c_str());

  RCLCPP_INFO(this->node_->get_logger(), "Shutting down unconstrained mpc controller");
  RCLCPP_INFO(this->node_->get_logger(), "unconstrained mpc controller shut down");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_error called. Previous state was '%s'",
    previous_state.label().c_str());

  RCLCPP_FATAL(this->node_->get_logger(), "An unknown error occurred. See the above log messages "
    "for more details. The controller will be finalized.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
UnconstrainedMpcController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cmd_iface_config;
  cmd_iface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  cmd_iface_config.names.insert(
    cmd_iface_config.names.end(),
    params_.required_hw_ifaces.plant.control_input.begin(),
    params_.required_hw_ifaces.plant.control_input.end());

  return cmd_iface_config;
}

controller_interface::InterfaceConfiguration
UnconstrainedMpcController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_iface_config;
  state_iface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_iface_config.names.insert(
    state_iface_config.names.end(),
    params_.required_hw_ifaces.plant.state.begin(),
    params_.required_hw_ifaces.plant.state.end());

  state_iface_config.names.insert(
    state_iface_config.names.end(),
    params_.required_hw_ifaces.plant.output.begin(),
    params_.required_hw_ifaces.plant.output.end());

  return state_iface_config;
}

controller_interface::return_type
UnconstrainedMpcController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::ignore = time;

  period_rt_pub_.getMsg().data = period.seconds();
  // period_rt_pub_.publish();

  robot_vel_vec_ = eigen_hw_ifaces_bridge_.plant_state->getStateInterfacesAsEigenVector();

  auto & robot_vel_msg = robot_vel_rt_pub_ptr_.getMsg();
  robot_vel_msg.linear.x = robot_vel_vec_(0);
  robot_vel_msg.linear.y = robot_vel_vec_(1);
  robot_vel_msg.linear.z = robot_vel_vec_(2);
  robot_vel_msg.angular.z = robot_vel_vec_(3);
  robot_vel_rt_pub_ptr_.publish();

  if (future_refs_rcvd_) {
    auto future_refs_values = future_refs_rt_buffer_.readFromRT();

    if (future_refs_values == nullptr) {
      RCLCPP_WARN(this->node_->get_logger(), "No future references received");
      return controller_interface::return_type::OK;
    }

    future_refs_ = future_refs_values->get()->data;
    future_refs_rcvd_ = false;

    size_t future_refs_size = future_refs_.size();
    size_t expected_future_refs_size = mpc_params_.output_size * mpc_params_.prediction_horizon;
    if (future_refs_size < expected_future_refs_size) {
      RCLCPP_WARN(
        this->node_->get_logger(),
        "Received future references vector has different size than expected. "
        "Expected >= %zu, Got: %zu. Values will be ignored.",
        expected_future_refs_size, future_refs_size);
      future_refs_.clear();
      return controller_interface::return_type::OK;
    }

    current_time_step_ = 0;

    plant_.state[kPreviousStep] =
      eigen_hw_ifaces_bridge_.plant_state->getStateInterfacesAsEigenVector();
    plant_.control_input[kPreviousStep] =
      eigen_hw_ifaces_bridge_.plant_control_input->getEigenVectorFromCommandInterfaces();

    RCLCPP_INFO(this->node_->get_logger(), "Future references received");

    return controller_interface::return_type::OK;
  }

  if (future_refs_.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->node_->get_logger(), *this->node_->get_clock(),
      kMillisecondToWarnNoFutureRefsRcvd, "No future references received");
    return controller_interface::return_type::OK;
  }

  plant_.state[kCurrentStep] =
    eigen_hw_ifaces_bridge_.plant_state->getStateInterfacesAsEigenVector();
  plant_.output =
    eigen_hw_ifaces_bridge_.plant_output->getStateInterfacesAsEigenVector();
  plant_.augmented_state =
    mpc_vectors_manipulator_->getAugmentedStateVector(
      plant_.state[kPreviousStep], plant_.state[kCurrentStep], plant_.output);

  horizon_refs_ = mpc_vectors_manipulator_->extractHorizonReferences(
    current_time_step_, future_refs_);

  plant_.control_input_increment =
    (ky_gain_ * horizon_refs_) - (kmpc_gain_ * plant_.augmented_state);

  plant_.control_input[kCurrentStep] = plant_.control_input[kPreviousStep] +
    plant_.control_input_increment;

  control_inputs_queue_.push(plant_.control_input[kCurrentStep]);

  if (control_inputs_queue_.size() > static_cast<size_t>(params_.control_input_delay_samples)) {
    eigen_hw_ifaces_bridge_.plant_control_input->setCommandInterfacesFromEigenVector(
      control_inputs_queue_.front());
    control_inputs_queue_.pop();
  }

  plant_.state[kPreviousStep] = plant_.state[kCurrentStep];
  plant_.control_input[kPreviousStep] = plant_.control_input[kCurrentStep];

  auto current_ref = future_refs_.begin() + (current_time_step_ * params_.state_size);
  if (current_ref < future_refs_.end()) {
    current_desired_robot_vel.insert(
      current_desired_robot_vel.begin(),
      current_ref,
      current_ref + params_.state_size);
  }

  auto & desired_robot_vel_msg = desired_robot_vel_rt_pub_ptr_.getMsg();
  desired_robot_vel_msg.linear.x = current_desired_robot_vel.at(0);
  desired_robot_vel_msg.linear.y = current_desired_robot_vel.at(1);
  desired_robot_vel_msg.linear.z = current_desired_robot_vel.at(2);
  desired_robot_vel_msg.angular.z = current_desired_robot_vel.at(3);
  desired_robot_vel_rt_pub_ptr_.publish();

  current_time_step_++;

  auto & ctrl_input_as_wrench = control_input_rt_pub_ptr_.getMsg();
  ctrl_input_as_wrench.force.x = plant_.control_input[kCurrentStep](0);
  ctrl_input_as_wrench.force.y = plant_.control_input[kCurrentStep](1);
  ctrl_input_as_wrench.force.z = plant_.control_input[kCurrentStep](2);
  ctrl_input_as_wrench.torque.z = plant_.control_input[kCurrentStep](3);
  control_input_rt_pub_ptr_.publish();

  return controller_interface::return_type::OK;
}

void UnconstrainedMpcController::resetPlantVectors()
{
  for (size_t i = 0; i < types::kNumberOfVectorsToStore; ++i) {
    plant_.state[i] = types::state_vector_t::Zero(mpc_params_.state_size);
    plant_.control_input[i] = types::control_vector_t::Zero(mpc_params_.control_size);
  }

  plant_.output = types::output_vector_t::Zero(mpc_params_.output_size);
  plant_.augmented_state = types::augmented_state_vector_t::Zero(
    mpc_params_.control_size + mpc_params_.output_size);
  plant_.control_input_increment = types::control_vector_t::Zero(mpc_params_.control_size);
}

}  // namespace unconstrained_mpc_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unconstrained_mpc_controller::UnconstrainedMpcController,
  controller_interface::ControllerInterface)
