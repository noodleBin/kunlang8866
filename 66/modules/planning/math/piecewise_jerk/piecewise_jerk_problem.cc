/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_problem.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

namespace {

constexpr double kMaxVariableRange = 1.0e10;
}  // namespace

using century::common::ErrorCode;
using century::common::Status;

PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots, const double delta_x,
    const std::array<double, 3>& x_init) {
  CHECK_GE(num_of_knots, 2U);
  CHECK_NE(delta_x, 0.0);
  num_of_knots_ = num_of_knots;

  x_init_ = x_init;

  delta_x_ = delta_x;

  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
}

OSQPData* PiecewiseJerkProblem::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  size_t kernel_dim = 3 * num_of_knots_;
  size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));
  data->q = CopyData(q);
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

Status PiecewiseJerkProblem::Optimize(const int max_iter) {
  OSQPData* data = FormulateProblem();

  OSQPSettings* settings = SolverDefaultSettings();
  if (nullptr == data || nullptr == settings) {
    std::stringstream ss_msg;
    ss_msg << "The data or settings for osqp is nullptr";
    AERROR << ss_msg.str();
    FreeData(data);
    c_free(settings);
    return Status(ErrorCode::PLANNING_ERROR, ss_msg.str());
  }
  settings->max_iter = max_iter;

  OSQPWorkspace* osqp_work = nullptr;
  osqp_work = osqp_setup(data, settings);
  // osqp_setup(&osqp_work, data, settings);
  if (nullptr == osqp_work) {
    std::stringstream ss_msg;
    ss_msg << "The osqp_work is nullptr";
    AERROR << ss_msg.str();
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return Status(ErrorCode::PLANNING_ERROR, ss_msg.str());
  }

  osqp_solve(osqp_work);

  auto status = osqp_work->info->status_val;

  std::stringstream ss_msg;
  if (status < 0 || (status != 1 && status != 2)) {
    ss_msg << "failed optimization status:\t" << osqp_work->info->status;
    AERROR << ss_msg.str();
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return Status(ErrorCode::PLANNING_ERROR, ss_msg.str());
  } else if (osqp_work->solution == nullptr) {
    ss_msg << "The solution from OSQP is nullptr";
    AERROR << ss_msg.str();
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return Status(ErrorCode::PLANNING_ERROR, ss_msg.str());
  }
  // AINFO << "osqp_work->info->iter=" << osqp_work->info->iter;

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
    ddx_.at(i) =
        osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
  }

  // Cleanup
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);
  return Status::OK();
}

void PiecewiseJerkProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 3N params bounds on x, x', x''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  int constraint_index = 0;

  ConstraintBoundary(n, num_of_variables, &constraint_index, &variables,
                     lower_bounds, upper_bounds);
  ConstraintContinuty(n, &constraint_index, &variables, lower_bounds,
                      upper_bounds);
  ConstraintXInit(n, &constraint_index, &variables, lower_bounds, upper_bounds);

  CHECK_EQ(constraint_index, num_of_constraints);
  StoreCSCMatrix(variables, num_of_variables, A_data, A_indices, A_indptr);
}
void PiecewiseJerkProblem::ConstraintBoundary(
    const int n, const int& num_of_variables, int* constraint_index_in,
    std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds) {
  // set x, x', x'' bounds
  if (nullptr == constraint_index_in || nullptr == variables ||
      nullptr == lower_bounds || nullptr == upper_bounds) {
    return;
  }
  int constraint_index = *constraint_index_in;
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      (*variables)[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      (*variables)[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      (*variables)[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }
  CHECK_EQ(constraint_index, num_of_variables);

  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_x
  for (int i = 0; i + 1 < n; ++i) {
    (*variables)[2 * n + i].emplace_back(constraint_index, -1.0);
    (*variables)[2 * n + i + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_x_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_x_ * scale_factor_[2];
    ++constraint_index;
  }
  *constraint_index_in = constraint_index;
}
void PiecewiseJerkProblem::ConstraintContinuty(
    const int n, int* constraint_index_in,
    std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds) {
  // x(i+1)' - x(i)' - 0.5 * delta_x * x(i)'' - 0.5 * delta_x * x(i+1)'' = 0
  if (nullptr == constraint_index_in || nullptr == variables ||
      nullptr == lower_bounds || nullptr == upper_bounds) {
    return;
  }
  int constraint_index = *constraint_index_in;
  for (int i = 0; i + 1 < n; ++i) {
    (*variables)[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    (*variables)[n + i + 1].emplace_back(constraint_index,
                                         1.0 * scale_factor_[2]);
    (*variables)[2 * n + i].emplace_back(constraint_index,
                                         -0.5 * delta_x_ * scale_factor_[1]);
    (*variables)[2 * n + i + 1].emplace_back(
        constraint_index, -0.5 * delta_x_ * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - delta_x * x(i)'
  // - 1/3 * delta_x^2 * x(i)'' - 1/6 * delta_x^2 * x(i+1)''
  auto delta_x_sq_ = delta_x_ * delta_x_;
  for (int i = 0; i + 1 < n; ++i) {
    (*variables)[i].emplace_back(constraint_index,
                                 -1.0 * scale_factor_[1] * scale_factor_[2]);
    (*variables)[i + 1].emplace_back(constraint_index,
                                     1.0 * scale_factor_[1] * scale_factor_[2]);
    (*variables)[n + i].emplace_back(
        constraint_index, -delta_x_ * scale_factor_[0] * scale_factor_[2]);
    (*variables)[2 * n + i].emplace_back(
        constraint_index,
        -delta_x_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    (*variables)[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_x_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }
  *constraint_index_in = constraint_index;
}
void PiecewiseJerkProblem::ConstraintXInit(
    const int& n, int* constraint_index_in,
    std::vector<std::vector<std::pair<c_int, c_float>>>* variables,
    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds) {
  if (nullptr == constraint_index_in || nullptr == variables ||
      nullptr == lower_bounds || nullptr == upper_bounds) {
    return;
  }
  int constraint_index = *constraint_index_in;
  // constrain on x_init,
  (*variables)[0].emplace_back(constraint_index, 1.0);

  (*lower_bounds)[constraint_index] = x_init_[0] * scale_factor_[0];
  (*upper_bounds)[constraint_index] = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  (*variables)[n].emplace_back(constraint_index, 1.0);
  (*lower_bounds)[constraint_index] = x_init_[1] * scale_factor_[1];
  (*upper_bounds)[constraint_index] = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  (*variables)[2 * n].emplace_back(constraint_index, 1.0);
  (*lower_bounds)[constraint_index] = x_init_[2] * scale_factor_[2];
  (*upper_bounds)[constraint_index] = x_init_[2] * scale_factor_[2];
  ++constraint_index;
  *constraint_index_in = constraint_index;
}
void PiecewiseJerkProblem::StoreCSCMatrix(
    const std::vector<std::vector<std::pair<c_int, c_float>>>& variables,
    const int& num_of_variables, std::vector<c_float>* A_data,
    std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr) {
  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      // coefficient
      A_data->push_back(variable_nz.second);

      // constraint index
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->push_back(ind_p);
}

OSQPSettings* PiecewiseJerkProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->verbose = FLAGS_enable_osqp_debug;
  settings->scaled_termination = true;
  return settings;
}

void PiecewiseJerkProblem::set_x_bounds(
    std::vector<std::pair<double, double>> x_bounds) {
  CHECK_EQ(x_bounds.size(), num_of_knots_);
  x_bounds_ = std::move(x_bounds);
}

void PiecewiseJerkProblem::set_dx_bounds(
    std::vector<std::pair<double, double>> dx_bounds) {
  CHECK_EQ(dx_bounds.size(), num_of_knots_);
  dx_bounds_ = std::move(dx_bounds);
}

void PiecewiseJerkProblem::set_ddx_bounds(
    std::vector<std::pair<double, double>> ddx_bounds) {
  CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  ddx_bounds_ = std::move(ddx_bounds);
}

void PiecewiseJerkProblem::set_x_bounds(const double x_lower_bound,
                                        const double x_upper_bound) {
  for (auto& x : x_bounds_) {
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }
}

void PiecewiseJerkProblem::set_dx_bounds(const double dx_lower_bound,
                                         const double dx_upper_bound) {
  for (auto& x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }
}

void PiecewiseJerkProblem::set_ddx_bounds(const double ddx_lower_bound,
                                          const double ddx_upper_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }
}

void PiecewiseJerkProblem::set_x_ref(const double weight_x_ref,
                                     std::vector<double> x_ref) {
  CHECK_EQ(x_ref.size(), num_of_knots_);
  weight_x_ref_ = weight_x_ref;
  // set uniform weighting
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, weight_x_ref);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

void PiecewiseJerkProblem::set_x_ref(std::vector<double> weight_x_ref_vec,
                                     std::vector<double> x_ref) {
  CHECK_EQ(x_ref.size(), num_of_knots_);
  CHECK_EQ(weight_x_ref_vec.size(), num_of_knots_);
  // set piecewise weighting
  weight_x_ref_vec_ = std::move(weight_x_ref_vec);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

void PiecewiseJerkProblem::set_end_state_ref(
    const std::array<double, 3>& weight_end_state,
    const std::array<double, 3>& end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ref_ = end_state_ref;
  has_end_state_ref_ = true;
}

void PiecewiseJerkProblem::InfoProblemParam() {
  AINFO << "*** num_of_knots ***";
  AINFO << "num_of_knots: " << num_of_knots_;

  AINFO << "*** x_init ***";
  AINFO << "x_init: (" << x_init_[0] << ", " << x_init_[1] << ", " << x_init_[2]
        << ")";
  AINFO << "*** scale_factor ***";
  AINFO << "scale_factor: (" << scale_factor_[0] << ", " << scale_factor_[1]
        << ", " << scale_factor_[2] << ")";

  AINFO << "*** weight ***";
  AINFO << "weight: (" << weight_x_ << ", " << weight_dx_ << ", " << weight_ddx_
        << ", " << weight_dddx_ << ")";

  AINFO << "*** delta_x ***";
  AINFO << "delta_x: " << delta_x_;

  AINFO << "*** end_state_ref ***";
  AINFO << "has_end_state_ref: " << has_end_state_ref_;
  AINFO << "end_state_ref: (" << end_state_ref_[0] << ", " << end_state_ref_[1]
        << ", " << end_state_ref_[2] << ")";
  AINFO << "weight_end_state: (" << weight_end_state_[0] << ", "
        << weight_end_state_[1] << ", " << weight_end_state_[2] << ")";

  AINFO << "*** x_ref ***";
  AINFO << "has_x_ref: " << has_x_ref_;
  AINFO << "weight_x_ref: " << weight_x_ref_;
  size_t index = 0UL;
  for (auto x_ref_point : x_ref_) {
    AINFO << "x_ref index(" << index++ << "): " << x_ref_point;
  }
  index = 0UL;
  for (auto x_ref_weight : weight_x_ref_vec_) {
    AINFO << "weight_x_ref_vec index(" << index++ << "): " << x_ref_weight;
  }

  AINFO << "*** x_bounds ***";
  index = 0UL;
  for (auto x_bounds_point : x_bounds_) {
    AINFO << "x_bounds index(" << index++ << "): [" << x_bounds_point.first
          << ", " << x_bounds_point.second << "]";
  }
  AINFO << "*** dx_bounds ***";
  index = 0UL;
  for (auto dx_bounds_point : dx_bounds_) {
    AINFO << "dx_bounds index(" << index++ << "): [" << dx_bounds_point.first
          << ", " << dx_bounds_point.second << "]";
  }
  AINFO << "*** ddx_bounds ***";
  index = 0UL;
  for (auto ddx_bounds_point : ddx_bounds_) {
    AINFO << "ddx_bounds index(" << index++ << "): [" << ddx_bounds_point.first
          << ", " << ddx_bounds_point.second << "]";
  }
  AINFO << "*** dddx_bound ***";
  AINFO << "dddx_bound: [" << dddx_bound_.first << ", " << dddx_bound_.second
        << "]";
}

void PiecewiseJerkProblem::FreeData(OSQPData* data) {
  delete[] data->q;
  data->q = nullptr;
  delete[] data->l;
  data->l = nullptr;
  delete[] data->u;
  data->u = nullptr;
  delete[] data->P->i;
  data->P->i = nullptr;
  delete[] data->P->p;
  data->P->p = nullptr;
  delete[] data->P->x;
  data->P->x = nullptr;
  delete[] data->A->i;
  data->A->i = nullptr;
  delete[] data->A->p;
  data->A->p = nullptr;
  delete[] data->A->x;
  data->A->x = nullptr;
}

}  // namespace planning
}  // namespace century
