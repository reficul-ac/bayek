#ifndef BAYEK_FSW_FAULT_H
#define BAYEK_FSW_FAULT_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

int bayek_fault_input_is_valid(const fsw_input_t *in);
fsw_mode_t bayek_fault_select_mode(const fsw_input_t *in, int input_valid);

#ifdef __cplusplus
}
#endif

#endif
