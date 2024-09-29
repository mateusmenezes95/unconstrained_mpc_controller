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

#include "unconstrained_mpc_controller/mpc_vectors_manipulator.hpp"

#include <iostream>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <vector>

#include "unconstrained_mpc_controller/types/mpc_definitions.hpp"
#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"

namespace unconstrained_mpc_controller
{

MpcVectorsManipulator::MpcVectorsManipulator(types::MpcParameters mpc_parameters)
{
  mpc_parameters.validate();
  mpc_params_ = mpc_parameters;

  first_diff_state_vec_ = types::state_vector_t::Zero(mpc_params_.state_size);
  augmented_state_vec_ = types::augmented_state_vector_t::Zero(
    mpc_params_.state_size + mpc_params_.output_size);

  future_refs_size_ = mpc_params_.output_size * mpc_params_.prediction_horizon;
  future_references_ = types::horizon_refs_vector_t::Zero(future_refs_size_);

  horizon_refs_vec_bridge_.resize(future_refs_size_);
  std::fill(horizon_refs_vec_bridge_.begin(), horizon_refs_vec_bridge_.end(), 0.0);
}

types::augmented_state_vector_t MpcVectorsManipulator::getAugmentedStateVector(
  const types::state_vector_t & previous_state_vec,
  const types::state_vector_t & current_state_vec,
  const types::output_vector_t & current_output_vec)
{
  first_diff_state_vec_ = current_state_vec - previous_state_vec;
  augmented_state_vec_ << first_diff_state_vec_, current_output_vec;
  return augmented_state_vec_;
}

types::horizon_refs_vector_t MpcVectorsManipulator::extractHorizonReferences(
  size_t current_time_step, const std::vector<double> & varying_time_references)
{
  auto horizon_start_it = varying_time_references.begin() +
    (current_time_step * mpc_params_.state_size);
  auto horizon_end_it = std::next(horizon_start_it, future_refs_size_);

  // If the horizon is greater than the varying time references, fill the future references
  // with the last reference
  if (horizon_end_it > varying_time_references.end())
  {
    auto begin_of_last_ref_it = std::prev(varying_time_references.end(), mpc_params_.state_size);
    auto end_of_last_ref_it = varying_time_references.end();

    auto tmp_it = future_references_.begin();
    for (size_t i = 0; i < mpc_params_.prediction_horizon; i++) {
      std::copy(begin_of_last_ref_it, end_of_last_ref_it, tmp_it);
      std::advance(tmp_it, mpc_params_.state_size);
    }

    return future_references_;
  }

  std::copy(horizon_start_it, horizon_end_it, future_references_.begin());

  return future_references_;
}

}  // namespace unconstrained_mpc_controller
