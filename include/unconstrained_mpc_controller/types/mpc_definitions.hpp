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

#include <fmt/format.h>

#include <algorithm>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace unconstrained_mpc_controller
{

namespace types
{

/**
 * @brief Helper struct to store the parameters of the MPC
 *
 * This struct assumes that the system is represented in state space form:
 * x(k+1) = Ax(k) + Bu(k)
 * y(k) = Cx(k) + Du(k)
 */
struct MpcParameters
{
  /**
   * @brief Quantity of states of the system (elements of x)
   */
  size_t state_size{0};

  /**
   * @brief Quantity of outputs of the system (elements of y)
   */
  size_t output_size{0};

  /**
   * @brief Quantity of control inputs of the system (elements of u)
   */
  size_t control_size{0};

  /**
   * @brief Prediction horizon of the MPC
   */
  size_t prediction_horizon{0};

  /**
   * @brief Control horizon of the MPC
   */
  size_t control_horizon{0};

  /**
   * @brief Sampling time of the system
   */
  double sampling_time{0.0};

  void validate() const
  {
    const std::map<std::string, size_t> parameters = {
        {"state_size", state_size},
        {"output_size", output_size},
        {"control_size", control_size},
        {"prediction_horizon", prediction_horizon},
        {"control_horizon", control_horizon},
    };

    std::vector<std::string> invalid_parameters;
    for (const auto & [parameter_name, parameter_value] : parameters)
    {
      if (parameter_value == 0)
      {
        invalid_parameters.push_back(parameter_name);
      }
    }

    if (!invalid_parameters.empty())
    {
      throw std::invalid_argument(fmt::format(
          "The following parameters must be greater than zero: {}",
          fmt::join(invalid_parameters, ", ")));
    }

    if (control_horizon > prediction_horizon)
    {
      throw std::invalid_argument(fmt::format(
          "The control horizon ({}) must be less than or equal to the prediction horizon ({}).",
          control_horizon,
          prediction_horizon));
    }

    if (sampling_time < 0.0)
    {
      throw std::invalid_argument("The sampling time must be greater than zero.");
    }
  }
};

}  // namespace types

}  // namespace unconstrained_mpc_controller
