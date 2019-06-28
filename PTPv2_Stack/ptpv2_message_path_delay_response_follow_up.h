/*
 * ptpv2_message_path_delay_response_follow_up.h
 *
 *  Created on: 5 lis 2016
 *      Author: Mat
 */

#ifndef PTPV2_MESSAGE_PATH_DELAY_RESPONSE_FOLLOW_UP_H_
#define PTPV2_MESSAGE_PATH_DELAY_RESPONSE_FOLLOW_UP_H_

typedef struct __attribute__((__packed__)) {
	ptpv2_message_common_header_t header;
	uint8_t receiveReceiptTimestamp[10];
	uint8_t requestingPortIdentity[10];
} ptpv2_message_path_delay_response_follow_up_t;

#endif /* PTPV2_MESSAGE_PATH_DELAY_RESPONSE_FOLLOW_UP_H_ */
