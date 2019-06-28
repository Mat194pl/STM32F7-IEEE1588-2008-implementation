/*
 * ptpv2_message_sync.h
 *
 *  Created on: 5 lis 2016
 *      Author: Mat
 */

#ifndef PTPV2_MESSAGE_SYNC_H_
#define PTPV2_MESSAGE_SYNC_H_

#include "ptpv2_header.h"

typedef struct __attribute__((__packed__)) {
	ptpv2_message_common_header_t header;
	uint8_t originTimestamp[10];
} ptpv2_message_sync_t;

#endif /* PTPV2_MESSAGE_SYNC_H_ */
