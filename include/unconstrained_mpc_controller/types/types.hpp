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

#include <array>
#include <cstddef>
#include <limits>

#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"

namespace unconstrained_mpc_controller
{

namespace types
{

constexpr size_t kInvalidIndex{std::numeric_limits<size_t>::max()};

/**
 * @brief Constant used in arrays that represents values at time step k-1 and k.
 *
 */
static constexpr size_t kNumberOfVectorsToStore{2};

/**
 * @brief Helper struct to store mappings between hardware interface indices and Eigen vector
 * indices.
 *
 * For example, if the index 0 of an eigen vector must store the value of the joint1/position state
 * interface that is at index 2 in the state interface vector, the hw_iface_idx would be 2 and the
 * eigen_vec_idx would be 0.
 */
struct EigenHwInterfaceIndexPair
{
  size_t hw_iface_idx{kInvalidIndex};
  size_t eigen_vec_idx{kInvalidIndex};
};

/**
 * @brief Helper struct to grout altogether values of the controller controlled plant modeled as a
 * discrete state-space system.
 * 
 */
struct PlantData {
  /**
   * @brief Helper array to store the last and current control input
   *
   * Element in index 0 is the control input at time k-1 and element in index 1 is the control input
   * at time k.
   */
  std::array<types::control_vector_t, kNumberOfVectorsToStore> control_input;

  /**
   * @brief Helper array to store the last and current state vector
   *
   * Element in index 0 is the state vector at time k-1 and element in index 1 is the state vector
   * at time k.
   */
  std::array<types::state_vector_t, kNumberOfVectorsToStore> state;

  /**
   * @brief Output vector at time k as variable to avoid reallocation of memory
   *
   */
  types::output_vector_t output;

  /**
   * @brief Augmented state vector at time k as variable to avoid reallocation of memory
   *
   */
  types::augmented_state_vector_t augmented_state;
};

}  // namespace types

}  // namespace unconstrained_mpc_controller
