/*
 * ptpv2_message_follow_up.h
 *
 *  Created on: 5 lis 2016
 *      Author: Mat
 */

#ifndef PTPV2_MESSAGE_FOLLOW_UP_H_
#define PTPV2_MESSAGE_FOLLOW_UP_H_

#include "ptpv2_header.h"

typedef struct __attribute__((__packed__)) {
	ptpv2_message_common_header_t header;
	uint8_t preciseOriginTimestamp[10];
} ptpv2_message_follow_up_t;

void ptpv2_message_follow_up_inverse_bytes(ptpv2_message_follow_up_t* message);

#endif /* PTPV2_MESSAGE_FOLLOW_UP_H_ */
