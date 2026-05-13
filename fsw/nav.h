#ifndef BAYEK_FSW_NAV_H
#define BAYEK_FSW_NAV_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void bayek_nav_reset(state_estimate_t *estimate);
void bayek_nav_update(const fsw_input_t *in, state_estimate_t *estimate);

#ifdef __cplusplus
}
#endif

#endif
