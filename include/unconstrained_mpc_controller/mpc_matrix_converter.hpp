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
#include <stdexcept>
#include <string_view>
#include <vector>

#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"
#include "unconstrained_mpc_controller/types/mpc_definitions.hpp"

namespace unconstrained_mpc_controller
{

/**
 * @brief Converts vectors and matrices to the format required by the MPC
 *
 */
class MpcMatrixConverter
{
public:
  /**
   * @brief Construct a new Mpc Matrix Converter object
   *
   * @param mpc_parameters The parameters of the MPC. @see MpcParameters
   * @throw std::invalid_argument If any of the parameters is zero or negative or the control
   * horizon is greater than the prediction horizon.
   */
  explicit MpcMatrixConverter(types::MpcParameters mpc_parameters);

  /**
   * @brief Converts a vector of doubles in row-major order to an Eigen matrix
   *
   * @param kmpc_gain_vec The vector of doubles to be converted. The size of the vector must be
   * equal to the control_size * state_size.
   * @return types::kmpc_gain_t The gain Kmpc in Eigen matrix format with control_size x state_size
   * dimensions.
   * @throw std::invalid_argument If the size of the input vector is different from control_size *
   * state_size.
   */
  types::kmpc_gain_t getKmpcGain(std::vector<double> & kmpc_gain_vec) const;

  /**
   * @brief Converts a vector of doubles in row-major order to an Eigen matrix
   *
   * @param ky_gain_vec The vector of doubles to be converted. The size of the vector must be equal
   * to the control_size * (output_size * prediction_horizon).
   * @return types::ky_gain_t The gain Ky in Eigen matrix format with control_size x (output_size *
   * prediction_horizon) dimensions.
   * @throw std::invalid_argument If the size of the input vector is different from control_size *
   * (output_size * prediction_horizon).
   */
  types::ky_gain_t getKyGain(std::vector<double> & ky_gain_vec) const;

  /**
   * @brief Default destructor
   *
   */
  virtual ~MpcMatrixConverter() = default;

private:
  /**
   * @brief Converts a vector of doubles in row-major order to an Eigen matrix
   *
   * @param vec The vector of doubles to be converted.
   * @param rows The number of rows of the matrix.
   * @param cols The number of columns of the matrix.
   * @param The name of the vector to be converted. This parameter is used in the exception message.
   * @return Eigen::MatrixXd The matrix with the values of the input vector.
   */
  Eigen::MatrixXd getMatrixFromVector(
    std::vector<double> & vec, size_t rows, size_t cols, std::string_view vec_name) const;

  /**
   * @brief Parameters of the MPC
   *
   */
  types::MpcParameters mpc_params_;
};

}  // namespace unconstrained_mpc_controller
