/*
 * ptpv2_helper.h
 *
 *  Created on: 5 lis 2016
 *      Author: Mat
 */

#ifndef PTPV2_HELPER_H_
#define PTPV2_HELPER_H_

#include "ptpv2_core.h"

typedef struct
{
	uint8_t sign;
	uint32_t nanoseconds;
	uint32_t seconds;
} ptpv2_time_stamp_t;

void ptpv2_helper_inverse_bytes(uint8_t* data, uint32_t length);
ptpv2_time_stamp_t ptpv2_time_stamp_add(ptpv2_time_stamp_t first, ptpv2_time_stamp_t second);
ptpv2_time_stamp_t ptpv2_time_stamp_sub(ptpv2_time_stamp_t first, ptpv2_time_stamp_t second);
ptpv2_time_stamp_t ptpv2_get_current_time(void);
void ptpv2_wait_for_tx_complete(void);
ptpv2_time_stamp_t ptpv2_get_last_tx_time_stamp(void);
void ptpv2_correct_system_clock(ptpv2_time_stamp_t offset);

#endif /* PTPV2_HELPER_H_ */
