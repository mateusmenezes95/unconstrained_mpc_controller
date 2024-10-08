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

#include "unconstrained_mpc_controller/eigen_vector_cmd_interface_bridge.hpp"

#include <eigen3/Eigen/Dense>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"

#include "unconstrained_mpc_controller/hw_interfaces_helper.hpp"

namespace unconstrained_mpc_controller
{

EigenVectorCmdInterfaceBridge::EigenVectorCmdInterfaceBridge(
  std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces,
  std::unordered_map<std::string, size_t> cmd_iface_to_eigen_vec_index)
: command_interfaces_(command_interfaces)
{
  index_map_ = HwInterfacesHelper::getIndexesPairs(
    command_interfaces, cmd_iface_to_eigen_vec_index);
}

void EigenVectorCmdInterfaceBridge::setCommandInterfacesFromEigenVector(
  const Eigen::VectorXd & eigen_vector) noexcept
{
  for (const auto & [_, index_pair] : index_map_)
  {
    command_interfaces_[index_pair.hw_iface_idx].set_value(
      eigen_vector(index_pair.eigen_vec_idx));
  }
}

Eigen::VectorXd EigenVectorCmdInterfaceBridge::getEigenVectorFromCommandInterfaces() const noexcept
{
  Eigen::VectorXd eigen_vec(index_map_.size());
  for (const auto & [_, index_pair] : index_map_)
  {
    eigen_vec(index_pair.eigen_vec_idx) = command_interfaces_[index_pair.hw_iface_idx].get_value();
  }
  return eigen_vec;
}

}  // namespace unconstrained_mpc_controller
