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
#include <vector>

#include "unconstrained_mpc_controller/types/mpc_definitions.hpp"
#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"

namespace unconstrained_mpc_controller
{

class MpcVectorsManipulator
{
public:
  /**
   * @brief Construct a new Mpc Vectors Manipulator object
   *
   * @param mpc_parameters The parameters of the MPC. @see MpcParameters
   * @throw std::invalid_argument If any of the parameters is zero or negative or the control
   * horizon is greater than the prediction horizon.
   */
  explicit MpcVectorsManipulator(types::MpcParameters mpc_parameters);

  /**
   * @brief Get the augmented state vector
   *
   * The augmented state vector id defined as: [deltax(k)', y(k)']
   *
   * @param previous_state_vec The state vector at time k-1
   * @param current_state_vec The state vector at time k
   * @param current_output_vec The output vector at time k
   * @return The augmented state vector
   */
  types::augmented_state_vector_t getAugmentedStateVector(
    const types::state_vector_t & previous_state_vec,
    const types::state_vector_t & current_state_vec,
    const types::output_vector_t & current_output_vec);

  /**
   * @brief Extract the future references in the prediction horizon
   * 
   * @param current_time_step Current time step k
   * @param varying_time_references The references that vary with time
   * @return types::horizon_refs_vector_t Vector with the future references in the prediction horizon
   */
  types::horizon_refs_vector_t extractHorizonReferences(
    size_t current_time_step, const std::vector<double> & varying_time_references);

private:
  /**
   * @brief Store the parameters of the MPC
   *
   * @see types::MpcParameters
   */
  types::MpcParameters mpc_params_;

  /**
   * @brief Store the first difference of the state vector
   *
   * i.e., deltax(k) = x(k) - x(k-1)
   *
   * @note Declared as a member variable to avoid reallocation of memory
   * in every call to getAugmentedStateVector and to be constructed only once.
   */
  types::state_vector_t first_diff_state_vec_;

  /**
   * @brief Store the augmented state vector
   *
   * i.e., [deltax(k)', y(k)']
   *
   * @note Declared as a member variable to avoid reallocation of memory
   * in every call to getAugmentedStateVector and to be constructed only once.
   */
  types::augmented_state_vector_t augmented_state_vec_;

  /**
   * @brief Store the future references in the prediction horizon
   *
   * @note Declared as a member variable to avoid reallocation of memory
   * in every call to getFutureReferences and to be constructed only once.
   */
  types::horizon_refs_vector_t future_references_;

  /**
   * @brief Store the size of the future references vector
   */
  size_t future_refs_size_{0};

  /**
   * @brief Intermediate vector to store the future references in the prediction horizon before
   * converting to Eigen vector
   * 
   */
  std::vector<double> horizon_refs_vec_bridge_;
};

}  // namespace unconstrained_mpc_controller
