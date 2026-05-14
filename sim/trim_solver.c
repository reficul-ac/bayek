#include "trim_solver.h"

#include "math_utils.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static real_t abs_real_local(real_t value) { return value < 0.0f ? -value : value; }

static real_t clamp_variable(real_t value, const bayek_trim_variable_t *variable) {
  return clamp_real(value, variable->min_value, variable->max_value);
}

static int variable_is_valid(const bayek_trim_variable_t *variable) {
  return variable != NULL && real_is_finite(variable->value) &&
         real_is_finite(variable->min_value) &&
         real_is_finite(variable->max_value) &&
         variable->min_value <= variable->max_value &&
         real_is_finite(variable->scale) && variable->scale > 0.0f &&
         real_is_finite(variable->perturbation) && variable->perturbation >= 0.0f;
}

static int residuals_are_valid(const real_t *residuals, uint32_t count) {
  uint32_t i;
  for (i = 0U; i < count; ++i) {
    if (!real_is_finite(residuals[i])) {
      return 0;
    }
  }
  return 1;
}

static real_t residual_norm(const real_t *residuals, uint32_t count) {
  real_t sum = 0.0f;
  uint32_t i;
  for (i = 0U; i < count; ++i) {
    sum += residuals[i] * residuals[i];
  }
  return (real_t)sqrtf(sum);
}

static int evaluate_at(const bayek_trim_problem_t *problem,
                       const bayek_trim_variable_t *variables,
                       real_t *residuals) {
  if (!problem->apply(problem, variables, problem->user)) {
    return 0;
  }
  if (!problem->evaluate(problem, residuals, problem->user)) {
    return 0;
  }
  return residuals_are_valid(residuals, problem->residual_count);
}

static int solve_linear(uint32_t n, real_t a[BAYEK_TRIM_MAX_VARIABLES][BAYEK_TRIM_MAX_VARIABLES],
                        real_t b[BAYEK_TRIM_MAX_VARIABLES],
                        real_t x[BAYEK_TRIM_MAX_VARIABLES]) {
  uint32_t i;
  uint32_t r;

  for (i = 0U; i < n; ++i) {
    uint32_t pivot = i;
    real_t pivot_abs = abs_real_local(a[i][i]);
    uint32_t j;
    for (r = i + 1U; r < n; ++r) {
      real_t candidate = abs_real_local(a[r][i]);
      if (candidate > pivot_abs) {
        pivot_abs = candidate;
        pivot = r;
      }
    }
    if (pivot_abs < 1.0e-8f) {
      return 0;
    }
    if (pivot != i) {
      real_t tmp_b = b[i];
      for (j = i; j < n; ++j) {
        real_t tmp = a[i][j];
        a[i][j] = a[pivot][j];
        a[pivot][j] = tmp;
      }
      b[i] = b[pivot];
      b[pivot] = tmp_b;
    }
    for (r = i + 1U; r < n; ++r) {
      real_t factor = a[r][i] / a[i][i];
      a[r][i] = 0.0f;
      for (j = i + 1U; j < n; ++j) {
        a[r][j] -= factor * a[i][j];
      }
      b[r] -= factor * b[i];
    }
  }

  for (i = n; i > 0U; --i) {
    uint32_t row = i - 1U;
    real_t sum = b[row];
    uint32_t c;
    for (c = row + 1U; c < n; ++c) {
      sum -= a[row][c] * x[c];
    }
    if (abs_real_local(a[row][row]) < 1.0e-8f) {
      return 0;
    }
    x[row] = sum / a[row][row];
    if (!real_is_finite(x[row])) {
      return 0;
    }
  }
  return 1;
}

static int problem_is_valid(const bayek_trim_problem_t *problem) {
  uint32_t i;
  if (problem == NULL || problem->variable_count == 0U ||
      problem->variable_count > BAYEK_TRIM_MAX_VARIABLES ||
      problem->residual_count == 0U ||
      problem->residual_count > BAYEK_TRIM_MAX_RESIDUALS ||
      problem->max_iterations == 0U || !real_is_finite(problem->tolerance) ||
      problem->tolerance <= 0.0f || !real_is_finite(problem->damping) ||
      problem->damping < 0.0f ||
      !real_is_finite(problem->finite_difference_step) ||
      problem->finite_difference_step <= 0.0f || problem->apply == NULL ||
      problem->evaluate == NULL) {
    return 0;
  }
  for (i = 0U; i < problem->variable_count; ++i) {
    if (!variable_is_valid(&problem->variables[i])) {
      return 0;
    }
  }
  return 1;
}

void bayek_trim_problem_default(bayek_trim_problem_t *problem) {
  if (problem == NULL) {
    return;
  }
  memset(problem, 0, sizeof(*problem));
  problem->max_iterations = 20U;
  problem->tolerance = 1.0e-4f;
  problem->damping = 1.0e-4f;
  problem->finite_difference_step = 1.0e-3f;
}

void bayek_trim_result_default(bayek_trim_result_t *result) {
  if (result == NULL) {
    return;
  }
  memset(result, 0, sizeof(*result));
  result->status = BAYEK_TRIM_STATUS_INVALID_PROBLEM;
}

const char *bayek_trim_status_name(bayek_trim_status_t status) {
  switch (status) {
  case BAYEK_TRIM_STATUS_CONVERGED:
    return "converged";
  case BAYEK_TRIM_STATUS_MAX_ITERATIONS:
    return "max_iterations";
  case BAYEK_TRIM_STATUS_INVALID_PROBLEM:
    return "invalid_problem";
  case BAYEK_TRIM_STATUS_INVALID_RESIDUAL:
    return "invalid_residual";
  case BAYEK_TRIM_STATUS_SINGULAR:
    return "singular";
  case BAYEK_TRIM_STATUS_NON_IMPROVING:
    return "non_improving";
  default:
    return "unknown";
  }
}

bayek_trim_status_t bayek_trim_solve(bayek_trim_problem_t *problem,
                                     bayek_trim_result_t *result) {
  bayek_trim_variable_t current[BAYEK_TRIM_MAX_VARIABLES];
  bayek_trim_variable_t trial[BAYEK_TRIM_MAX_VARIABLES];
  real_t residual[BAYEK_TRIM_MAX_RESIDUALS];
  real_t trial_residual[BAYEK_TRIM_MAX_RESIDUALS];
  real_t jacobian[BAYEK_TRIM_MAX_RESIDUALS][BAYEK_TRIM_MAX_VARIABLES];
  real_t normal[BAYEK_TRIM_MAX_VARIABLES][BAYEK_TRIM_MAX_VARIABLES];
  real_t rhs[BAYEK_TRIM_MAX_VARIABLES];
  real_t step[BAYEK_TRIM_MAX_VARIABLES];
  real_t norm = 0.0f;
  bayek_trim_status_t status = BAYEK_TRIM_STATUS_MAX_ITERATIONS;
  uint32_t iter = 0U;
  uint32_t n;
  uint32_t m;

  if (result != NULL) {
    bayek_trim_result_default(result);
  }
  if (!problem_is_valid(problem)) {
    return BAYEK_TRIM_STATUS_INVALID_PROBLEM;
  }

  n = problem->variable_count;
  m = problem->residual_count;
  memcpy(current, problem->variables, sizeof(current));
  for (uint32_t i = 0U; i < n; ++i) {
    current[i].value = clamp_variable(current[i].value, &current[i]);
  }

  if (!evaluate_at(problem, current, residual)) {
    status = BAYEK_TRIM_STATUS_INVALID_RESIDUAL;
    goto finish;
  }
  norm = residual_norm(residual, m);

  for (iter = 0U; iter < problem->max_iterations; ++iter) {
    if (problem->report != NULL) {
      problem->report(problem, iter, norm, current, problem->user);
    }
    if (norm <= problem->tolerance) {
      status = BAYEK_TRIM_STATUS_CONVERGED;
      goto finish_with_iter;
    }

    for (uint32_t c = 0U; c < n; ++c) {
      real_t delta = current[c].perturbation > 0.0f
                         ? current[c].perturbation
                         : problem->finite_difference_step * current[c].scale;
      real_t actual_delta;
      memcpy(trial, current, sizeof(trial));
      trial[c].value = clamp_variable(current[c].value + delta, &current[c]);
      actual_delta = trial[c].value - current[c].value;
      if (abs_real_local(actual_delta) < 1.0e-8f) {
        trial[c].value = clamp_variable(current[c].value - delta, &current[c]);
        actual_delta = trial[c].value - current[c].value;
      }
      if (abs_real_local(actual_delta) < 1.0e-8f) {
        for (uint32_t r = 0U; r < m; ++r) {
          jacobian[r][c] = 0.0f;
        }
        continue;
      }
      if (!evaluate_at(problem, trial, trial_residual)) {
        status = BAYEK_TRIM_STATUS_INVALID_RESIDUAL;
        goto finish_with_iter;
      }
      for (uint32_t r = 0U; r < m; ++r) {
        jacobian[r][c] = (trial_residual[r] - residual[r]) / actual_delta;
      }
    }

    memset(normal, 0, sizeof(normal));
    memset(rhs, 0, sizeof(rhs));
    memset(step, 0, sizeof(step));
    for (uint32_t r = 0U; r < m; ++r) {
      for (uint32_t c = 0U; c < n; ++c) {
        rhs[c] -= jacobian[r][c] * residual[r];
        for (uint32_t k = 0U; k < n; ++k) {
          normal[c][k] += jacobian[r][c] * jacobian[r][k];
        }
      }
    }
    for (uint32_t c = 0U; c < n; ++c) {
      normal[c][c] += problem->damping / (current[c].scale * current[c].scale);
    }
    if (!solve_linear(n, normal, rhs, step)) {
      status = BAYEK_TRIM_STATUS_SINGULAR;
      goto finish_with_iter;
    }

    {
      real_t best_norm = norm;
      int improved = 0;
      real_t alpha = 1.0f;
      for (uint32_t attempt = 0U; attempt < 10U; ++attempt) {
        memcpy(trial, current, sizeof(trial));
        for (uint32_t c = 0U; c < n; ++c) {
          trial[c].value = clamp_variable(current[c].value + alpha * step[c], &current[c]);
        }
        if (!evaluate_at(problem, trial, trial_residual)) {
          status = BAYEK_TRIM_STATUS_INVALID_RESIDUAL;
          goto finish_with_iter;
        }
        best_norm = residual_norm(trial_residual, m);
        if (best_norm < norm) {
          improved = 1;
          break;
        }
        alpha *= 0.5f;
      }
      if (!improved) {
        status = BAYEK_TRIM_STATUS_NON_IMPROVING;
        goto finish_with_iter;
      }
      memcpy(current, trial, sizeof(current));
      memcpy(residual, trial_residual, sizeof(residual));
      norm = best_norm;
    }
  }

  iter = problem->max_iterations;
finish_with_iter:
  if (status == BAYEK_TRIM_STATUS_CONVERGED) {
    memcpy(problem->variables, current, sizeof(problem->variables));
  }
finish:
  if (result != NULL) {
    result->status = status;
    result->iteration_count = iter;
    result->residual_norm = norm;
    memcpy(result->variables, current, sizeof(result->variables));
  }
  return status;
}
