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

#include <eigen3/Eigen/Dense>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"

#include "unconstrained_mpc_controller/hw_interfaces_helper.hpp"
#include "unconstrained_mpc_controller/types/types.hpp"

namespace unconstrained_mpc_controller
{

class EigenVectorCmdInterfaceBridge
{
public:
  /**
   * @brief Construct a new object to bridge the Eigen vector with the command interfaces.
   * 
   * @param command_interfaces Loaned command interfaces to set the values from the Eigen vector.
   * @param cmd_iface_to_eigen_vec_index Map of command interface names to the corresponding indexes
   * in the Eigen vector.
   * @throws std::runtime_error If a command interface name in the map is not found in the command
   * interfaces vector.
   */
  EigenVectorCmdInterfaceBridge(
    std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces,
    std::unordered_map<std::string, size_t> cmd_iface_to_eigen_vec_index);

  /**
   * @brief Default destructor.
   * 
   */
  virtual ~EigenVectorCmdInterfaceBridge() = default;

  /**
   * @brief Set the command interfaces with the values from the Eigen vector.
   *
   * @param cmd_vector The Eigen vector with the values to set the command interfaces.
   */
  void setCommandInterfacesFromEigenVector(const Eigen::VectorXd & cmd_vector) noexcept;

  /**
   * @brief Get the command interfaces as an Eigen vector.
   *
   * @return Eigen::VectorXd The command interfaces as an Eigen vector.
   */
  Eigen::VectorXd getEigenVectorFromCommandInterfaces() const noexcept;

private:
  /**
   * @brief Holds the command interfaces for the controller.
   *
   * The command interfaces are used to send commands to the hardware. For this controller, the
   * command interfaces will be set with the values from the control signal computed by the
   * unconstrained MPC.
   */
  std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces_;

  /**
   * @brief Maps the command interface names to the corresponding indexes in the command_interfaces_
   * vector and the corresponding indexes in the Eigen output vector.
   */
  std::unordered_map<std::string, types::EigenHwInterfaceIndexPair> index_map_;
};

}  // namespace unconstrained_mpc_controller
