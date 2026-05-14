#ifndef BAYEK_TRIM_SOLVER_H
#define BAYEK_TRIM_SOLVER_H

#include "common_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BAYEK_TRIM_MAX_VARIABLES 16U
#define BAYEK_TRIM_MAX_RESIDUALS 16U

typedef enum {
  BAYEK_TRIM_STATUS_CONVERGED = 0,
  BAYEK_TRIM_STATUS_MAX_ITERATIONS,
  BAYEK_TRIM_STATUS_INVALID_PROBLEM,
  BAYEK_TRIM_STATUS_INVALID_RESIDUAL,
  BAYEK_TRIM_STATUS_SINGULAR,
  BAYEK_TRIM_STATUS_NON_IMPROVING
} bayek_trim_status_t;

typedef struct {
  const char *name;
  real_t value;
  real_t min_value;
  real_t max_value;
  real_t scale;
  real_t perturbation;
} bayek_trim_variable_t;

typedef struct bayek_trim_problem bayek_trim_problem_t;

typedef int (*bayek_trim_apply_fn)(const bayek_trim_problem_t *problem,
                                   const bayek_trim_variable_t *variables,
                                   void *user);
typedef int (*bayek_trim_residual_fn)(const bayek_trim_problem_t *problem,
                                      real_t *residuals,
                                      void *user);
typedef void (*bayek_trim_report_fn)(const bayek_trim_problem_t *problem,
                                     uint32_t iteration,
                                     real_t residual_norm,
                                     const bayek_trim_variable_t *variables,
                                     void *user);

struct bayek_trim_problem {
  uint32_t variable_count;
  uint32_t residual_count;
  uint32_t max_iterations;
  real_t tolerance;
  real_t damping;
  real_t finite_difference_step;
  bayek_trim_variable_t variables[BAYEK_TRIM_MAX_VARIABLES];
  void *user;
  bayek_trim_apply_fn apply;
  bayek_trim_residual_fn evaluate;
  bayek_trim_report_fn report;
};

typedef struct {
  bayek_trim_status_t status;
  uint32_t iteration_count;
  real_t residual_norm;
  bayek_trim_variable_t variables[BAYEK_TRIM_MAX_VARIABLES];
} bayek_trim_result_t;

void bayek_trim_problem_default(bayek_trim_problem_t *problem);
void bayek_trim_result_default(bayek_trim_result_t *result);
const char *bayek_trim_status_name(bayek_trim_status_t status);
bayek_trim_status_t bayek_trim_solve(bayek_trim_problem_t *problem,
                                     bayek_trim_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
