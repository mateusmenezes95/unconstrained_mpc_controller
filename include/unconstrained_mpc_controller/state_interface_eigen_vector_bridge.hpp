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

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"

#include "unconstrained_mpc_controller/hw_interfaces_helper.hpp"
#include "unconstrained_mpc_controller/types/types.hpp"

namespace unconstrained_mpc_controller
{

class StateInterfaceEigenVectorBridge
{
public:
  StateInterfaceEigenVectorBridge(
    std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
    std::unordered_map<std::string, size_t> & state_iface_to_eigen_vec_index);
  virtual ~StateInterfaceEigenVectorBridge() = default;

private:
  /**
   * @brief Holds the state interfaces for the controller.
   *
   * The state interfaces are used to read the current state of the hardware. For this controller,
   * the state interfaces will be used to read the current states and outputs of the plant to be used
   * as inputs for the unconstrained MPC.
   */
  std::vector<hardware_interface::LoanedStateInterface> & state_interfaces_;

  /**
   * @brief Maps the state interface names to the corresponding indexes in the state_interfaces_
   * vector and the corresponding indexes in the Eigen state vector.
   */
  std::unordered_map<std::string, types::EigenHwInterfaceIndexPair> index_map_;
};

}  // namespace unconstrained_mpc_controller
