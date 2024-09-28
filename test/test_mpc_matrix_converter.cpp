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

#include <iostream>
#include <stdexcept>
#include <vector>

#include "unconstrained_mpc_controller/mpc_matrix_converter.hpp"
#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"

TEST(MpcMatrixConverter, Constructor_throws_if_any_parameter_is_zero)
{
  unconstrained_mpc_controller::types::MpcParameters mpc_parameters;
  mpc_parameters.state_size = 1;
  mpc_parameters.output_size = 1;
  mpc_parameters.control_size = 1;
  mpc_parameters.prediction_horizon = 1;
  mpc_parameters.control_horizon = 1;

  EXPECT_NO_THROW(unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters));

  mpc_parameters.state_size = 0;
  EXPECT_THROW(unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters),
    std::invalid_argument);

  mpc_parameters.state_size = 1;
  mpc_parameters.output_size = 0;
  EXPECT_THROW(unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters),
    std::invalid_argument);

  mpc_parameters.output_size = 1;
  mpc_parameters.control_size = 0;
  EXPECT_THROW(unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters),
    std::invalid_argument);

  mpc_parameters.control_size = 1;
  mpc_parameters.prediction_horizon = 0;
  EXPECT_THROW(unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters),
    std::invalid_argument);

  mpc_parameters.prediction_horizon = 1;
  mpc_parameters.control_horizon = 0;
  EXPECT_THROW(unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters),
    std::invalid_argument);
}

TEST(MpcMatrixConverter, Constructor_throws_if_control_horizon_is_greater_than_prediction_horizon)
{
  unconstrained_mpc_controller::types::MpcParameters mpc_parameters;
  mpc_parameters.state_size = 1;
  mpc_parameters.output_size = 1;
  mpc_parameters.control_size = 1;
  mpc_parameters.prediction_horizon = 1;
  mpc_parameters.control_horizon = 1;

  EXPECT_NO_THROW(unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters));

  mpc_parameters.control_horizon = 2;
  EXPECT_THROW(unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters),
    std::invalid_argument);
}

TEST(MpcMatrixConverter, Get_mpc_gain_fails_if_input_vector_has_invalid_size)
{
  unconstrained_mpc_controller::types::MpcParameters mpc_parameters;
  mpc_parameters.state_size = 2;
  mpc_parameters.output_size = 1;
  mpc_parameters.control_size = 3;
  mpc_parameters.prediction_horizon = 1;
  mpc_parameters.control_horizon = 1;

  unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters);

  std::vector<double> kmpc_gain_vec = {1.0, 2.0, 3.0, 4.0, 5.0};

  try {
    converter.getKmpcGain(kmpc_gain_vec);
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    std::cout << e.what() << std::endl;
    SUCCEED();
  }
}

TEST(MpcMatrixConverter, Get_ky_gain_fails_if_input_vector_has_invalid_size)
{
  unconstrained_mpc_controller::types::MpcParameters mpc_parameters;
  mpc_parameters.state_size = 2;
  mpc_parameters.output_size = 3;
  mpc_parameters.control_size = 3;
  mpc_parameters.prediction_horizon = 1;
  mpc_parameters.control_horizon = 1;

  unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters);

  std::vector<double> ky_gain_vec = {1.0, 2.0, 3.0, 4.0, 5.0};

  try {
    converter.getKyGain(ky_gain_vec);
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    std::cout << e.what() << std::endl;
    SUCCEED();
  }
}

TEST(MpcMatrixConverter, Get_kmpc_gain_returns_correct_matrix)
{
  unconstrained_mpc_controller::types::MpcParameters mpc_parameters;
  mpc_parameters.state_size = 4;
  mpc_parameters.output_size = 4;
  mpc_parameters.control_size = 4;
  mpc_parameters.prediction_horizon = 3;
  mpc_parameters.control_horizon = 3;

  unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters);

  std::vector<double> kmpc_gain_vec =
    {
      1.0, 2.0, 3.0, 4.0,
      5.0, 6.0, 7.0, 8.0,
      9.0, 10.0, 11.0, 12.0,
      13.0, 14.0, 15.0, 16.0
    };

  auto kmpc_matrix = converter.getKmpcGain(kmpc_gain_vec);

  EXPECT_EQ(kmpc_matrix.rows(), mpc_parameters.control_size);
  EXPECT_EQ(kmpc_matrix.cols(), mpc_parameters.state_size);

  EXPECT_EQ(kmpc_matrix(0, 0), 1.0);
  EXPECT_EQ(kmpc_matrix(0, 1), 2.0);
  EXPECT_EQ(kmpc_matrix(0, 2), 3.0);
  EXPECT_EQ(kmpc_matrix(0, 3), 4.0);
  EXPECT_EQ(kmpc_matrix(1, 0), 5.0);
  EXPECT_EQ(kmpc_matrix(1, 1), 6.0);
  EXPECT_EQ(kmpc_matrix(1, 2), 7.0);
  EXPECT_EQ(kmpc_matrix(1, 3), 8.0);
  EXPECT_EQ(kmpc_matrix(2, 0), 9.0);
  EXPECT_EQ(kmpc_matrix(2, 1), 10.0);
  EXPECT_EQ(kmpc_matrix(2, 2), 11.0);
  EXPECT_EQ(kmpc_matrix(2, 3), 12.0);
  EXPECT_EQ(kmpc_matrix(3, 0), 13.0);
  EXPECT_EQ(kmpc_matrix(3, 1), 14.0);
  EXPECT_EQ(kmpc_matrix(3, 2), 15.0);
  EXPECT_EQ(kmpc_matrix(3, 3), 16.0);
}

TEST(MpcMatrixConverter, Get_ky_gain_returns_correct_matrix)
{
  unconstrained_mpc_controller::types::MpcParameters mpc_parameters;
  mpc_parameters.state_size = 4;
  mpc_parameters.output_size = 4;
  mpc_parameters.control_size = 4;
  mpc_parameters.prediction_horizon = 3;
  mpc_parameters.control_horizon = 3;

  unconstrained_mpc_controller::MpcMatrixConverter converter(mpc_parameters);

  std::vector ky_gain_vec =
  {
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0
  };

  auto ky_matrix = converter.getKyGain(ky_gain_vec);

  EXPECT_EQ(ky_matrix.rows(), mpc_parameters.control_size);
  EXPECT_EQ(ky_matrix.cols(), mpc_parameters.output_size * mpc_parameters.prediction_horizon);

  EXPECT_EQ(ky_matrix(0, 0), 1.0);
  EXPECT_EQ(ky_matrix(0, 1), 2.0);
  EXPECT_EQ(ky_matrix(0, 2), 3.0);
  EXPECT_EQ(ky_matrix(0, 3), 4.0);
  EXPECT_EQ(ky_matrix(0, 4), 5.0);
  EXPECT_EQ(ky_matrix(0, 5), 6.0);
  EXPECT_EQ(ky_matrix(0, 6), 7.0);
  EXPECT_EQ(ky_matrix(0, 7), 8.0);
  EXPECT_EQ(ky_matrix(0, 8), 9.0);
  EXPECT_EQ(ky_matrix(0, 9), 10.0);
  EXPECT_EQ(ky_matrix(0, 10), 11.0);
  EXPECT_EQ(ky_matrix(0, 11), 12.0);

  EXPECT_EQ(ky_matrix(1, 0), 1.0);
  EXPECT_EQ(ky_matrix(1, 1), 2.0);
  EXPECT_EQ(ky_matrix(1, 2), 3.0);
  EXPECT_EQ(ky_matrix(1, 3), 4.0);
  EXPECT_EQ(ky_matrix(1, 4), 5.0);
  EXPECT_EQ(ky_matrix(1, 5), 6.0);
  EXPECT_EQ(ky_matrix(1, 6), 7.0);
  EXPECT_EQ(ky_matrix(1, 7), 8.0);
  EXPECT_EQ(ky_matrix(1, 8), 9.0);
  EXPECT_EQ(ky_matrix(1, 9), 10.0);
  EXPECT_EQ(ky_matrix(1, 10), 11.0);
  EXPECT_EQ(ky_matrix(1, 11), 12.0);

  EXPECT_EQ(ky_matrix(2, 0), 1.0);
  EXPECT_EQ(ky_matrix(2, 1), 2.0);
  EXPECT_EQ(ky_matrix(2, 2), 3.0);
  EXPECT_EQ(ky_matrix(2, 3), 4.0);
  EXPECT_EQ(ky_matrix(2, 4), 5.0);
  EXPECT_EQ(ky_matrix(2, 5), 6.0);
  EXPECT_EQ(ky_matrix(2, 6), 7.0);
  EXPECT_EQ(ky_matrix(2, 7), 8.0);
  EXPECT_EQ(ky_matrix(2, 8), 9.0);
  EXPECT_EQ(ky_matrix(2, 9), 10.0);
  EXPECT_EQ(ky_matrix(2, 10), 11.0);
  EXPECT_EQ(ky_matrix(2, 11), 12.0);
}
