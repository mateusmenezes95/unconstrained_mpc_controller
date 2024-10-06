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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
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
    return controller_interface::CallbackReturn::ERROR;
  }

  plant_.output = types::output_vector_t::Zero(params_.output_size);

  auto check_interfaces = [this](const auto & iface_names, size_t required_size,
    const std::string & iface_type) {
      if (iface_names.size() < required_size) {
        throw std::runtime_error(fmt::format(
          "Not enough {} interfaces provided. Required at least {}, provided {}",
          iface_type, required_size, iface_names.size()));
    }
  };

  try {
    check_interfaces(
      params_.required_hw_ifaces.plant.control_input, params_.control_size, "command");
    check_interfaces(
      params_.required_hw_ifaces.plant.state, params_.state_size, "state");
    check_interfaces(
      params_.required_hw_ifaces.plant.output, params_.output_size, "output");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  auto future_refs_callback = [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!subscriber_is_active_) {
      RCLCPP_WARN(
        this->node_->get_logger(),
        "Received future references but subscriber is not active. "
        "Data will be ignored.");
      return;
    }

    future_refs_rt_buffer_.writeFromNonRT(msg);
  };

  try {
    future_refs_sub_ = this->get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/future_refs", 10, future_refs_callback);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->node_->get_logger(), "Configuration of unconstrained mpc controller succeeded");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_cleanup called. Previous state was '%s'",
    previous_state.label().c_str());

  RCLCPP_INFO(this->node_->get_logger(), "Cleaning up unconstrained mpc controller");

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

  auto create_iface_to_index_map = [](const auto& iface_names) {
      std::unordered_map<std::string, size_t> iface_to_index_map;
      for (size_t i = 0; i < iface_names.size(); ++i) {
          iface_to_index_map[iface_names[i]] = i;
      }
      return iface_to_index_map;
  };

  try {
    eigen_vec_cmd_iface_bridge_ = std::make_unique<EigenVectorCmdInterfaceBridge>(
      this->command_interfaces_,
      create_iface_to_index_map(params_.required_hw_ifaces.plant.control_input));
    state_iface_eigen_vec_bridge_ = std::make_unique<StateInterfaceEigenVectorBridge>(
      this->state_interfaces_,
      create_iface_to_index_map(params_.required_hw_ifaces.plant.state));
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


  RCLCPP_INFO(this->node_->get_logger(), "unconstrained mpc controller deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_shutdown called. Previous state was '%s'",
    previous_state.label().c_str());

  RCLCPP_INFO(this->node_->get_logger(), "Shut down unconstrained mpc controller");


  RCLCPP_INFO(this->node_->get_logger(), "unconstrained mpc controller shut down");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnconstrainedMpcController::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(this->node_->get_logger(), "on_error called. Previous state was '%s'",
    previous_state.label().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
UnconstrainedMpcController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cmd_iface_config;
  cmd_iface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return cmd_iface_config;
}

controller_interface::InterfaceConfiguration
UnconstrainedMpcController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_iface_config;
  state_iface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return state_iface_config;
}

controller_interface::return_type
UnconstrainedMpcController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (future_refs_rcvd_) {
    auto future_refs_rcvd = future_refs_rt_buffer_.readFromRT();

    if (!future_refs_rcvd) {
      RCLCPP_ERROR(this->node_->get_logger(), "Realtime buffer is empty");
      return controller_interface::return_type::ERROR;
    }

    future_refs_rcvd_ = false;
    current_time_step_ = 0;

    future_refs_ = future_refs_rcvd->get()->data;

    size_t future_refs_size = future_refs_.size();
    size_t expected_future_refs_size = mpc_params_.output_size * mpc_params_.prediction_horizon;
    if (future_refs_size != expected_future_refs_size) {
      RCLCPP_WARN(
        this->node_->get_logger(),
        "Received future references vector has different size than expected. "
        "Expected: %zu, Got: %zu. Values will be ignored.",
        expected_future_refs_size, future_refs_size);
      future_refs_.clear();
      return controller_interface::return_type::OK;
    }

    RCLCPP_INFO(this->node_->get_logger(), "Future references received");
  } else {
    current_time_step_++;
  }

  if (future_refs_.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->node_->get_logger(), *this->node_->get_clock(),
      kMillisecondToWarnNoFutureRefsRcvd, "No future references received");
      return controller_interface::return_type::OK;
  }

  if (current_time_step_ == 0) {
    plant_.state[kPreviousStep] = state_iface_eigen_vec_bridge_->getStateInterfacesAsEigenVector();
    plant_.control_input[kPreviousStep] =
      eigen_vec_cmd_iface_bridge_->getEigenVectorFromCommandInterfaces();
    return controller_interface::return_type::OK;
  }

  return controller_interface::return_type::OK;
}

void UnconstrainedMpcController::reset()
{
  subscriber_is_active_ = false;
  future_refs_sub_.reset();  // Reset the pointer stops the subscription
  future_refs_sub_.reset();

  future_refs_rcvd_ = false;
  future_refs_.clear();
}

}  // namespace unconstrained_mpc_controller
