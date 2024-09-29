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

#include <eigen3/Eigen/Dense>
#include <gmock/gmock.h>
#include <fmt/format.h>

#include "unconstrained_mpc_controller/mpc_vectors_manipulator.hpp"
#include "unconstrained_mpc_controller/types/mpc_definitions.hpp"
#include "unconstrained_mpc_controller/types/mpc_matrices.hpp"

constexpr double kEpsilon{1e-6};

TEST(TestMpcVectorsManipulator, Get_successfuly_augmented_state_vector)
{
  unconstrained_mpc_controller::types::MpcParameters mpc_parameters;
  mpc_parameters.state_size = 6;
  mpc_parameters.output_size = 6;
  mpc_parameters.control_size = 6;
  mpc_parameters.prediction_horizon = 10;
  mpc_parameters.control_horizon = 5;
  mpc_parameters.sampling_time = 0.01;

  EXPECT_NO_THROW(mpc_parameters.validate());

  unconstrained_mpc_controller::MpcVectorsManipulator mpc_vectors_manipulator(mpc_parameters);

  unconstrained_mpc_controller::types::state_vector_t previous_state_vec(6);
  previous_state_vec << 1, 2, 3, 4, 5, 6;

  unconstrained_mpc_controller::types::state_vector_t current_state_vec(6);
  current_state_vec << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6;

  unconstrained_mpc_controller::types::output_vector_t current_output_vec(6);
  current_output_vec << 1, 2, 3, 4, 5, 6;

  unconstrained_mpc_controller::types::augmented_state_vector_t augmented_state(12);

  augmented_state = mpc_vectors_manipulator.getAugmentedStateVector(previous_state_vec,
    current_state_vec, current_output_vec);

  EXPECT_EQ(augmented_state.size(), 12);

  EXPECT_NEAR(augmented_state(0), 0.1, kEpsilon);
  EXPECT_NEAR(augmented_state(1), 0.2, kEpsilon);
  EXPECT_NEAR(augmented_state(2), 0.3, kEpsilon);
  EXPECT_NEAR(augmented_state(3), 0.4, kEpsilon);
  EXPECT_NEAR(augmented_state(4), 0.5, kEpsilon);
  EXPECT_NEAR(augmented_state(5), 0.6, kEpsilon);
  EXPECT_NEAR(augmented_state(6), 1, kEpsilon);
  EXPECT_NEAR(augmented_state(7), 2, kEpsilon);
  EXPECT_NEAR(augmented_state(8), 3, kEpsilon);
  EXPECT_NEAR(augmented_state(9), 4, kEpsilon);
  EXPECT_NEAR(augmented_state(10), 5, kEpsilon);
  EXPECT_NEAR(augmented_state(11), 6, kEpsilon);
}

struct StepSizeAndHorizonRefs
{
  StepSizeAndHorizonRefs(size_t step_size_arg, std::vector<double> horizon_refs_arg)
  : step_size(step_size_arg), horizon_refs(horizon_refs_arg) {}
  size_t step_size;
  std::vector<double> horizon_refs;
};

std::vector<StepSizeAndHorizonRefs> expected_horizon_refs = {
  StepSizeAndHorizonRefs{0, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
  StepSizeAndHorizonRefs{1, {3, 4, 5, 6, 7, 8, 9, 10, 11, 12}},
  StepSizeAndHorizonRefs{2, {5, 6, 7, 8, 9, 10, 11, 12, 13, 14}},
  StepSizeAndHorizonRefs{3, {7, 8, 9, 10, 11, 12, 13, 14, 15, 16}},
  StepSizeAndHorizonRefs{4, {9, 10, 11, 12, 13, 14, 15, 16, 17, 18}},
  StepSizeAndHorizonRefs{5, {11, 12, 13, 14, 15, 16, 17, 18, 19, 20}},
  StepSizeAndHorizonRefs{6, {13, 14, 15, 16, 17, 18, 19, 20, 19, 20}},
  StepSizeAndHorizonRefs{7, {15, 16, 17, 18, 19, 20, 19, 20, 19, 20}},
  StepSizeAndHorizonRefs{8, {17, 18, 19, 20, 19, 20, 19, 20, 19, 20}},
  StepSizeAndHorizonRefs{9, {19, 20, 19, 20, 19, 20, 19, 20, 19, 20}},
  StepSizeAndHorizonRefs{10, {19, 20, 19, 20, 19, 20, 19, 20, 19, 20}},
};

struct TestFixtureForStepSizesThatExceedsPredictionHorizon
  : public testing::TestWithParam<StepSizeAndHorizonRefs>
{
  void SetUp() override
  {
    mpc_parameters.state_size = 2;
    mpc_parameters.output_size = 2;
    mpc_parameters.control_size = 2;
    mpc_parameters.prediction_horizon = 5;
    mpc_parameters.control_horizon = 5;
    mpc_parameters.sampling_time = 0.01;

    EXPECT_NO_THROW(mpc_parameters.validate());

    mpc_vectors_manipulator = std::make_unique<unconstrained_mpc_controller::MpcVectorsManipulator>(
      mpc_parameters);

    for (size_t i = 0; i < 20; i++) {
      varying_time_references.push_back(static_cast<double>(i + 1));
    }
  }

  unconstrained_mpc_controller::types::MpcParameters mpc_parameters;
  std::unique_ptr<unconstrained_mpc_controller::MpcVectorsManipulator> mpc_vectors_manipulator;
  std::vector<double> varying_time_references;
};

TEST_P(TestFixtureForStepSizesThatExceedsPredictionHorizon, Get_successfuly_future_references)
{
  size_t current_time_step = GetParam().step_size;
  unconstrained_mpc_controller::types::horizon_refs_vector_t future_references =
    mpc_vectors_manipulator->extractHorizonReferences(current_time_step, varying_time_references);

  size_t future_references_size = mpc_parameters.state_size * mpc_parameters.prediction_horizon;
  EXPECT_EQ(future_references.size(), future_references_size);

  for (size_t i = 0; i < future_references_size; i++) {
    EXPECT_NEAR(future_references(i), GetParam().horizon_refs[i], kEpsilon);
  }
}

INSTANTIATE_TEST_SUITE_P(
  TestMpcVectorsManipulator,
  TestFixtureForStepSizesThatExceedsPredictionHorizon,
  testing::ValuesIn(expected_horizon_refs),
  [](const testing::TestParamInfo<TestFixtureForStepSizesThatExceedsPredictionHorizon::ParamType> &
  info_arg) {
    return fmt::format("StepSize_{}", info_arg.param.step_size);
  });
