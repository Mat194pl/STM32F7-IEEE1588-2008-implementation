/*
 * ptpv2_common.h
 *
 *  Created on: 22 paü 2016
 *      Author: Mat
 */

#ifndef PTPV2_COMMON_H_
#define PTPV2_COMMON_H_

#include "ptpv2_conf.h"

#define PTPV2_HEADER_MESSAGE_TYPE_SYNC 0x0
#define PTPV2_HEADER_MESSAGE_TYPE_DELAY_REQUEST 0x1
#define PTPV2_HEADER_MESSAGE_TYPE_PATH_DELAY_REQUEST 0x2
#define PTPV2_HEADER_MESSAGE_TYPE_PATH_DELAY_RESPONSE 0x3
#define PTPV2_HEADER_MESSAGE_TYPE_FOLLOW_UP 0x8
#define PTPV2_HEADER_MESSAGE_TYPE_DELAY_RESPONSE 0x9
#define PTPV2_HEADER_MESSAGE_TYPE_PATH_DELAY_RESPONSE_FOLLOW_UP 0xa
#define PTPV2_HEADER_MESSAGE_TYPE_ANNOUNCE 0xb
#define PTPV2_HEADER_MESSAGE_TYPE_SIGNALING 0xc
#define PTPV2_HEADER_MESSAGE_TYPE_MANAGEMENT 0xd

typedef struct
{
	void (*inverse_bytes_func)(void*);
	const uint32_t messageLength;
	const uint32_t messageType;
	void (*receive_callback_func)(void*);
} ptpv2_message_descriptor_t;

#endif /* PTPV2_COMMON_H_ */
