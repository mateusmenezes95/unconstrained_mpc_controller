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

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "unconstrained_mpc_controller/hw_interfaces_helper.hpp"

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

  hardware_interface::StateInterface state_interface_0{"state_interface", "index_0"};
  hardware_interface::StateInterface state_interface_1{"state_interface", "index_1"};
  hardware_interface::StateInterface state_interface_2{"state_interface", "index_2"};
  hardware_interface::CommandInterface cmd_interface_0{"cmd_interface", "index_0"};
  hardware_interface::CommandInterface cmd_interface_1{"cmd_interface", "index_1"};
  hardware_interface::CommandInterface cmd_interface_2{"cmd_interface", "index_2"};
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  std::vector<hardware_interface::LoanedCommandInterface> cmd_interfaces;
};

TEST_F(TestFixture, Get_interface_by_name_fails_with_invalid_name)
{
  ASSERT_THROW(
    unconstrained_mpc_controller::HwInterfacesHelper::getInterfaceIndexByName(
    "state_interface/invalid_name", state_interfaces), std::runtime_error);
  ASSERT_THROW(
    unconstrained_mpc_controller::HwInterfacesHelper::getInterfaceIndexByName(
    "command_interface/invalid_name", cmd_interfaces), std::runtime_error);
}

TEST_F(TestFixture, Get_interface_by_name_succeeds_with_valid_name)
{
  std::string state_interface_name = "state_interface/index_";
  std::string cmd_interface_name = "cmd_interface/index_";
  for (size_t i = 0; i < state_interfaces.size(); i++) {
    ASSERT_EQ(i, unconstrained_mpc_controller::HwInterfacesHelper::getInterfaceIndexByName(
      state_interface_name + std::to_string(i), state_interfaces));
    ASSERT_EQ(i, unconstrained_mpc_controller::HwInterfacesHelper::getInterfaceIndexByName(
      cmd_interface_name + std::to_string(i), cmd_interfaces));
  }
}
