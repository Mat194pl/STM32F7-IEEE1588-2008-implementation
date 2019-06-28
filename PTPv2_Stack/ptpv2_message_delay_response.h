/*
 * ptpv2_message_delay_response.h
 *
 *  Created on: 5 lis 2016
 *      Author: Mat
 */

#ifndef PTPV2_MESSAGE_DELAY_RESPONSE_H_
#define PTPV2_MESSAGE_DELAY_RESPONSE_H_

#include "ptpv2_header.h"

typedef struct __attribute__((__packed__)) {
	ptpv2_message_common_header_t header;
	uint8_t receiveTimestamp[10];
	uint8_t requestingPortIdentity[10];
} ptpv2_message_delay_response_t;

void ptpv2_message_delay_response_inverse_bytes(ptpv2_message_delay_response_t* message);

#endif /* PTPV2_MESSAGE_DELAY_RESPONSE_H_ */
