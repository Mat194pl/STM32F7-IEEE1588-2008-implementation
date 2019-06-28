/*
 * ptpv2_message_follow_up.c
 *
 *  Created on: 6 lis 2016
 *      Author: Mat
 */

#include "ptpv2_message_follow_up.h"
#include "ptpv2_helper.h"

extern volatile uint32_t lastFollowUpHighTimeStamp;
extern volatile uint32_t lastFollowUpLowTimeStamp;
extern volatile uint8_t hasFollowUp;
extern void ptpv2_slave_send_delay_request(void);

void ptpv2_message_follow_up_inverse_bytes(ptpv2_message_follow_up_t* message);
void ptpv2_message_follow_up_receive_callback(ptpv2_message_follow_up_t* message);

ptpv2_message_descriptor_t PTPV2_MESSAGE_FOLLOW_UP = {
		.inverse_bytes_func = (void(*)(void*))ptpv2_message_follow_up_inverse_bytes,
		.messageLength = 44,
		.messageType = PTPV2_HEADER_MESSAGE_TYPE_FOLLOW_UP,
		.receive_callback_func = (void(*)(void*))ptpv2_message_follow_up_receive_callback
};


void ptpv2_message_follow_up_inverse_bytes(ptpv2_message_follow_up_t* message)
{
	ptpv2_helper_inverse_bytes((uint8_t*)&message->preciseOriginTimestamp, 10);
}

void ptpv2_message_follow_up_receive_callback(ptpv2_message_follow_up_t* message)
{
	memcpy(&lastFollowUpLowTimeStamp, &(message->preciseOriginTimestamp[0]), 4);
	memcpy(&lastFollowUpHighTimeStamp, &(message->preciseOriginTimestamp[4]), 4);
	 ptpv2_slave_send_delay_request();

	hasFollowUp = 1;
	//LOG("Received FollowUp: Low: %X, High: %X\n", lastFollowUpLowTimeStamp, lastFollowUpHighTimeStamp);
}
