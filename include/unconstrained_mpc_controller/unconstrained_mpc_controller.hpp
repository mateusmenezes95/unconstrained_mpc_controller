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

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <queue>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "unconstrained_mpc_controller_params.hpp"  // NOLINT
#include "unconstrained_mpc_controller/eigen_vector_cmd_interface_bridge.hpp"
#include "unconstrained_mpc_controller/mpc_matrix_converter.hpp"
#include "unconstrained_mpc_controller/mpc_vectors_manipulator.hpp"
#include "unconstrained_mpc_controller/realtime_publisher_wrapper.hpp"
#include "unconstrained_mpc_controller/state_interface_eigen_vector_bridge.hpp"
#include "unconstrained_mpc_controller/types/mpc_definitions.hpp"
#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"
#include "unconstrained_mpc_controller/types/types.hpp"

namespace unconstrained_mpc_controller
{

static constexpr size_t kMillisecondToWarnNoFutureRefsRcvd{1000};

/**
 * @brief Represent the discrete time step k-1 in a array with only two elements.
 * 
 */
static constexpr size_t kPreviousStep{0};

/**
 * @brief Represent the discrete time step k in a array with only two elements.
 * 
 */
static constexpr size_t kCurrentStep{1};

/**
 * @brief Unconstrained MPC controller under the ros2_control framework concept.
 *
 * This class is a controller interface that implements the unconstrained MPC controller. It apply
 * the MPC control law to the system and send the control signal to the system through the hardware
 * interfaces.
 */
class UnconstrainedMpcController : public controller_interface::ControllerInterface
{
public:
  /**
   * @brief Default constructor implementation.
   *
   */
  UnconstrainedMpcController() = default;

  /**
   * @brief Default destructor implementation.
   *
   */
  virtual ~UnconstrainedMpcController() = default;

  /**
   * @brief Initialize the controller.
   *
   * Loads and validate the controller parameters and initializes the controller.
   *
   * @return controller_interface::CallbackReturn::SUCCESS if the controller was initialized
   * successfully, otherwise controller_interface::CallbackReturn::ERROR.
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Configure the controller
   *
   * Retrieves the controller parameters and fill internal variables to be used by the unconstrained
   * MPC control law.
   *
   * @param previous_state State of the controller before this transition.
   * @retval controller_interface::CallbackReturn::SUCCESS if the controller was configured with
   * success
   * @retval controller_interface::CallbackReturn::ERROR if some error that needs some sort of
   * recovery procedure
   * @retval controller_interface::CallbackReturn::FAILURE if the controller was not configured
   * with success and should not proceed to the next state
   */
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Cleanup the controller.
   *
   * @param previous_state State of the controller before this transition.
   * @retval controller_interface::CallbackReturn::SUCCESS if the controller was configured with
   * success
   * @retval controller_interface::CallbackReturn::ERROR if some error that needs some sort of
   * recovery procedure
   * @retval controller_interface::CallbackReturn::FAILURE if the controller was not configured
   * with success and should not proceed to the next state
   */
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate the controller.
   *
   * Activates ROS interfaces
   *
   * @param previous_state State of the controller before this transition.
   * @retval controller_interface::CallbackReturn::SUCCESS if the controller was configured with
   * success
   * @retval controller_interface::CallbackReturn::ERROR if some error that needs some sort of
   * recovery procedure
   * @retval controller_interface::CallbackReturn::FAILURE if the controller was not configured
   * with success and should not proceed to the next state
   */
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the controller
   *
   * Deactivates ROS interfaces and clear the future references of the Unconstrained MPC
   *
   * @param previous_state State of the controller before this transition.
   * @retval controller_interface::CallbackReturn::SUCCESS if the controller was configured with
   * success
   * @retval controller_interface::CallbackReturn::ERROR if some error that needs some sort of
   * recovery procedure
   * @retval controller_interface::CallbackReturn::FAILURE if the controller was not configured
   * with success and should not proceed to the next state
   */
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Shutdown the controller
   *
   * @param previous_state State of the controller before this transition.
   * @retval controller_interface::CallbackReturn::SUCCESS if the controller was shutdown with
   * success
   * @retval controller_interface::CallbackReturn::ERROR if some error that needs some sort of
   * recovery procedure
   * @retval controller_interface::CallbackReturn::FAILURE if the controller was not shutdown
   * with success and should not proceed to the next state
   */
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Try to recover the controller from an error state
   *
   * @param previous_state
   * @return controller_interface::CallbackReturn::SUCCESS If the controller was recovered with
   * success
   * @return controller_interface::CallbackReturn::ERROR If the controller must be shutdown
   */
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Defines which command interfaces are needed by the controller.
   *
   * In the case of the unconstrained MPC controller, commands interfaces are populated by the
   * control signal values computed by the MPC control law without any constraints.
   *
   * @return controller_interface::InterfaceConfiguration Struct containing the name of the
   * command interfaces needed by the controller.
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Defines which state interfaces are needed by the controller.
   *
   * In the case of the unconstrained MPC controller, state interfaces are populated by the system
   * states that are used by the MPC control law to compute the control signal.
   *
   * @return controller_interface::InterfaceConfiguration Struct containing the name of the state
   * interfaces needed by the controller.
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Compute the control signal using the MPC control law without constraints and fill the
   * command interfaces.
   *
   * **The method called in the (real-time) control loop.**
   *
   * \param time The time at the start of this control loop iteration
   * \param period The measured time taken by the last control loop iteration
   * \returns return_type::OK if update is successfully, otherwise return_type::ERROR.
   */
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /**
   * @brief Reset the values of all plant vectors in @ref plant_ to zero.
   * 
   */
  void resetPlantVectors();

  /**
   * @brief Holds the node that will be used by the controller.
   *
   */
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  /**
   * @brief Listener provided by the generated_parameters_library to get the controller parameters.
   *
   */
  std::shared_ptr<ParamListener> param_listener_;

  /**
   * @brief Controller parameters.
   *
   * This struct contains the parameters needed by the unconstrained MPC controller to compute the
   * control signal. The parameters are automatically generated by the generate_parameters_library.
   * See the unconstrained_mpc_controller_params.yaml file in the src folder for more details.
   */
  Params params_;

  /**
   * @brief Struct that holds the parameters of the unconstrained MPC control law implementation.
   *
   */
  types::MpcParameters mpc_params_;

  /**
   * @brief Converter that transforms std::vector to MPC maprices
   *
   */
  std::unique_ptr<MpcMatrixConverter> mpc_matrix_converter_;

  /**
   * @brief Manipulator that handles the MPC vectors
   *
   */
  std::unique_ptr<MpcVectorsManipulator> mpc_vectors_manipulator_;

  /**
   * @brief Struct that holds the Eigen bridges to the hardware interfaces.
   *
   */
  struct EigenHwInterfaceBridges {
    /**
     * @brief Eigen vector bridge that connects the computed control input to the command interfaces
     *
     */
    std::unique_ptr<EigenVectorCmdInterfaceBridge> plant_control_input;

    /**
     * @brief Eigen vector bridge that connects the plant state to the state interfaces
     *
     */
    std::unique_ptr<StateInterfaceEigenVectorBridge> plant_state;

    /**
     * @brief Eigen vector bridge that connects the plant output to the state interfaces
     *
     */
    std::unique_ptr<StateInterfaceEigenVectorBridge> plant_output;
  } eigen_hw_ifaces_bridge_;

  /**
   * @brief Unconstrained MPC gain matrix that multiplies the augmented state vector to obtain the
   * control vector.
   *
   */
  types::kmpc_gain_t kmpc_gain_;

  /**
   * @brief Unconstrained MPC gain matrix that multiplies the future references to obtain the control
   * vector.
   *
   */
  types::ky_gain_t ky_gain_;

  /**
   * @brief Future references that the system should track.
   *
   */
  std::vector<double> future_refs_;

  /**
   * @brief Vector that holds the future references in the prediction horizon.
   *
   */
  types::horizon_refs_vector_t horizon_refs_;

  /**
   * @brief Struct that holds all plant data used by the MPC control law.
   * 
   */
  types::PlantData plant_;

  /**
   * @brief Atomic flag to check if the subscriber is active.
   *
   */
  std::atomic_bool subscriber_is_active_{false};

  /**
   * @brief Subscriber to the future references topic.
   *
   */
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr future_refs_sub_;

  /**
   * @brief Realtime buffer to store the future references.
   *
   */
  realtime_tools::RealtimeBuffer<std_msgs::msg::Float64MultiArray::SharedPtr>
  future_refs_rt_buffer_;

  // TODO(mmeneses): Move publisher for independent package. This implementation must be generic
  // and agnostic to the robot/plant being used
  types::state_vector_t robot_vel_vec_;
  RealtimePublisherWrapper<geometry_msgs::msg::Wrench> control_input_rt_pub_ptr_;
  RealtimePublisherWrapper<geometry_msgs::msg::Twist> robot_vel_rt_pub_ptr_;
  RealtimePublisherWrapper<geometry_msgs::msg::Twist> desired_robot_vel_rt_pub_ptr_;
  RealtimePublisherWrapper<std_msgs::msg::Float64> period_rt_pub_;
  std::queue<types::control_vector_t> control_inputs_queue_;
  std::vector<double> current_desired_robot_vel;

  /**
   * @brief Flag to check if the future references were received.
   *
   */
  std::atomic_bool future_refs_rcvd_{false};

  /**
   * @brief Current time step of the MPC control law.
   *
   * This is the "k" index of the discrete time system.
   */
  std::atomic<size_t> current_time_step_{0};
};

}  // namespace unconstrained_mpc_controller
