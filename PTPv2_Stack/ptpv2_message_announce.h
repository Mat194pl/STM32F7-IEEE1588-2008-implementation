/*
 * ptpv2_announce_message.h
 *
 *  Created on: 22 paü 2016
 *      Author: Mat
 */

#ifndef PTPV2_MESSAGE_ANNOUNCE_H_
#define PTPV2_MESSAGE_ANNOUNCE_H_

#include "ptpv2_header.h"

typedef struct __attribute__((__packed__)) {
	ptpv2_message_common_header_t header;
	uint8_t originTimestamp[10];
	uint16_t currentUtcOffset;
	uint8_t reserved;
	uint8_t grandmasterPriority1;
	uint32_t grandmasterClockQuality;
	uint8_t grandmasterPriority2;
	uint8_t grandmasterIdentity[8];
	uint16_t stepsRemoved;
	uint8_t timeSource;
} ptpv2_message_announce_t;

//void ptpv2_message_announce_inverse_bytes(ptpv2_message_announce_t* message);

#endif /* PTPV2_MESSAGE_ANNOUNCE_H_ */
