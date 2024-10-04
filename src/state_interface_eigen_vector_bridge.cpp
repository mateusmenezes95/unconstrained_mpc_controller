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

#include "unconstrained_mpc_controller/state_interface_eigen_vector_bridge.hpp"

#include <eigen3/Eigen/Dense>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"

#include "unconstrained_mpc_controller/hw_interfaces_helper.hpp"

namespace unconstrained_mpc_controller
{

StateInterfaceEigenVectorBridge::StateInterfaceEigenVectorBridge(
  std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
  std::unordered_map<std::string, size_t> & state_iface_to_eigen_vec_index)
  : state_interfaces_(state_interfaces)
{
  index_map_ = HwInterfacesHelper::getIndexesPairs(
    state_interfaces, state_iface_to_eigen_vec_index);
}

Eigen::VectorXd StateInterfaceEigenVectorBridge::getStateInterfacesAsEigenVector() const noexcept
{
  Eigen::VectorXd eigen_vector(index_map_.size());
  for (const auto & [_, index_pair] : index_map_)
  {
    eigen_vector(index_pair.eigen_vec_idx) = state_interfaces_[index_pair.hw_iface_idx].get_value();
  }

  return eigen_vector;
}

}  // namespace unconstrained_mpc_controller
