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

#include <array>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "unconstrained_mpc_controller/types/types.hpp"
#include "unconstrained_mpc_controller/eigen_vector_cmd_interface_bridge.hpp"
#include "unconstrained_mpc_controller/state_interface_eigen_vector_bridge.hpp"

struct TestFixture : public ::testing::Test
{
  void SetUp() override
  {
    state_interfaces.emplace_back(std::ref(state_interface_0));
    state_interfaces.emplace_back(std::ref(state_interface_1));
    state_interfaces.emplace_back(std::ref(state_interface_2));

    cmd_interfaces.emplace_back(std::ref(cmd_interface_0));
    cmd_interfaces.emplace_back(std::ref(cmd_interface_1));
    cmd_interfaces.emplace_back(std::ref(cmd_interface_2));
  }

  std::unordered_map<std::string, size_t> cmd_iface_to_eigen_vec_indexes{
    {"cmd_interface/index_0", 0},
    {"cmd_interface/index_1", 1},
    {"cmd_interface/index_2", 2}
  };

  std::array<double, 3> state_interface_values{0.0, 0.0, 0.0};
  hardware_interface::StateInterface state_interface_0{
    "state_interface", "index_0", &state_interface_values[0]};
  hardware_interface::StateInterface state_interface_1{
    "state_interface", "index_1", &state_interface_values[1]};
  hardware_interface::StateInterface state_interface_2{
    "state_interface", "index_2", &state_interface_values[2]};

  std::unordered_map<std::string, size_t> state_iface_to_eigen_vec_indexes{
    {"state_interface/index_0", 0},
    {"state_interface/index_1", 1},
    {"state_interface/index_2", 2}
  };

  std::array<double, 3> cmd_interface_values{0.0, 0.0, 0.0};
  hardware_interface::CommandInterface cmd_interface_0{
    "cmd_interface", "index_0", &cmd_interface_values[0]};
  hardware_interface::CommandInterface cmd_interface_1{
    "cmd_interface", "index_1", &cmd_interface_values[1]};
  hardware_interface::CommandInterface cmd_interface_2{
    "cmd_interface", "index_2", &cmd_interface_values[2]};

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  std::vector<hardware_interface::LoanedCommandInterface> cmd_interfaces;
};

TEST_F(TestFixture, Construction_fails_if_cmd_iface_name_is_invalid)
{
  std::unordered_map<std::string, size_t> cmd_iface_to_eigen_vec_index;
  cmd_iface_to_eigen_vec_index["cmd_interface/invalid_name"] = 0;

  try {
    unconstrained_mpc_controller::EigenVectorCmdInterfaceBridge bridge(
      cmd_interfaces, cmd_iface_to_eigen_vec_index);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    std::cerr << e.what() << std::endl;
  }
}

TEST_F(TestFixture, Construction_fails_if_state_iface_name_is_invalid)
{
  std::unordered_map<std::string, size_t> state_iface_to_eigen_vec_index;
  state_iface_to_eigen_vec_index["state_interface/invalid_name"] = 0;

  try {
    unconstrained_mpc_controller::StateInterfaceEigenVectorBridge bridge(
      state_interfaces, state_iface_to_eigen_vec_index);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    std::cerr << e.what() << std::endl;
  }
}

TEST_F(TestFixture, Construction_succeeds_if_cmd_iface_name_is_valid)
{
  unconstrained_mpc_controller::EigenVectorCmdInterfaceBridge bridge(
    this->cmd_interfaces, this->cmd_iface_to_eigen_vec_indexes);
}

TEST_F(TestFixture, Construction_succeeds_if_state_iface_name_is_valid)
{
  unconstrained_mpc_controller::StateInterfaceEigenVectorBridge bridge(
    this->state_interfaces, this->state_iface_to_eigen_vec_indexes);
}

TEST_F(TestFixture, Set_command_interfaces_from_eigen_vector)
{
  unconstrained_mpc_controller::EigenVectorCmdInterfaceBridge bridge(
    this->cmd_interfaces, this->cmd_iface_to_eigen_vec_indexes);

  Eigen::VectorXd cmd_vector(3);
  cmd_vector << 1.0, 2.0, 3.0;

  bridge.setCommandInterfacesFromEigenVector(cmd_vector);

  ASSERT_EQ(cmd_interface_values[0], cmd_vector[0]);
  ASSERT_EQ(cmd_interface_values[1], cmd_vector[1]);
  ASSERT_EQ(cmd_interface_values[2], cmd_vector[2]);
}

TEST_F(TestFixture, Get_state_interfaces_as_eigen_vector)
{
  unconstrained_mpc_controller::StateInterfaceEigenVectorBridge bridge(
    this->state_interfaces, this->state_iface_to_eigen_vec_indexes);

  state_interface_values[0] = 1.0;
  state_interface_values[1] = 2.0;
  state_interface_values[2] = 3.0;

  Eigen::VectorXd state_vector = bridge.getStateInterfacesAsEigenVector();

  ASSERT_EQ(state_interface_values[0], state_vector[0]);
  ASSERT_EQ(state_interface_values[1], state_vector[1]);
  ASSERT_EQ(state_interface_values[2], state_vector[2]);
}
