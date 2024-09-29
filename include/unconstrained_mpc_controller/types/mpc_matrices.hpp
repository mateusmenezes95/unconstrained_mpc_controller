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

namespace unconstrained_mpc_controller
{

namespace types
{

/**
 * @brief Gain matrix that multiplies the state vector to obtain the control
 * vector.
 *
 * @note Notation used in the book WANG, L. Model Predictive Control System
 * Design and Implementation Using MATLAB. 1st ed.
 */
typedef Eigen::MatrixXd kmpc_gain_t;

/**
 * @brief Gain matrix that multiplies the future references to obtain the control vector.
 *
 * @note Notation used in the book WANG, L. Model Predictive Control System
 * Design and Implementation Using MATLAB. 1st ed.
 */
typedef Eigen::MatrixXd ky_gain_t;

/**
 * @brief State vector type.
 *
 */
typedef Eigen::VectorXd state_vector_t;

/**
 * @brief Output vector type.
 *
 */
typedef Eigen::VectorXd output_vector_t;

/**
 * @brief Augmented state vector type.
 *
 */
typedef Eigen::VectorXd augmented_state_vector_t;

/**
 * @brief Control vector type.
 *
 */
typedef Eigen::VectorXd control_vector_t;

/**
 * @brief Vector with the future references in the prediction horizon.
 *
 */
typedef Eigen::VectorXd horizon_refs_vector_t;

}  // namespace types

}  // namespace unconstrained_mpc_controller

