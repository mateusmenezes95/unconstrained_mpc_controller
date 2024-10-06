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

#include "unconstrained_mpc_controller/mpc_matrix_converter.hpp"

#include <eigen3/Eigen/Dense>
#include <fmt/format.h>

#include <exception>
#include <stdexcept>
#include <vector>

#include "unconstrained_mpc_controller/types/mpc_definitions.hpp"
#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"

namespace unconstrained_mpc_controller
{

MpcMatrixConverter::MpcMatrixConverter(types::MpcParameters mpc_parameters)
{
  mpc_parameters.validate();
  mpc_params_ = mpc_parameters;
}

types::kmpc_gain_t MpcMatrixConverter::getKmpcGain(
  std::vector<double> & kmpc_gain_vector) const
{
  return this->getMatrixFromVector(
    kmpc_gain_vector,
    mpc_params_.control_size,
    mpc_params_.state_size + mpc_params_.control_size,
    "kmpc gains");
}

types::ky_gain_t MpcMatrixConverter::getKyGain(
  std::vector<double> & ky_gain_vector) const
{
  return this->getMatrixFromVector(
    ky_gain_vector,
    mpc_params_.control_size,
    mpc_params_.output_size * mpc_params_.prediction_horizon,
    "ky gains");
}

Eigen::MatrixXd MpcMatrixConverter::getMatrixFromVector(
  std::vector<double> & matrix_vector, size_t rows, size_t cols, std::string_view vec_name) const
{
  if (matrix_vector.size() != rows * cols) {
    throw std::invalid_argument(fmt::format(
      "The size ({}) of the vector with {} must be equal {} (rows * cols = {} * {})",
      matrix_vector.size(),
      vec_name,
      rows * cols,
      rows,
      cols));
  }

  // Convert the vector to a matrix using the row-major order. See the Eigen::Map documentation for
  // more details.
  auto matrix = Eigen::Map<
    Eigen::MatrixXd,
    Eigen::Unaligned,
    Eigen::Stride<1, Eigen::Dynamic>>(
    matrix_vector.data(),
    rows,
    cols,
    Eigen::Stride<1, Eigen::Dynamic>(1, cols));

  return matrix;
}

}  // namespace unconstrained_mpc_controller
