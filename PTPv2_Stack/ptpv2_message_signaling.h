/*
 * ptpv2_message_signalling.h
 *
 *  Created on: 5 lis 2016
 *      Author: Mat
 */

#ifndef PTPV2_MESSAGE_SIGNALING_H_
#define PTPV2_MESSAGE_SIGNALING_H_

#include "ptpv2_header.h"

#define PTPV2_MESSAGE_SIGNALING_PAYLOAD_MAX_LENGTH 255

typedef struct __attribute__((__packed__)) {
	ptpv2_message_common_header_t header;
	uint8_t targetPortIdentity[10];
	uint8_t payload[PTPV2_MESSAGE_SIGNALING_PAYLOAD_MAX_LENGTH];
} ptpv2_message_signaling_t;

#endif /* PTPV2_MESSAGE_SIGNALING_H_ */
