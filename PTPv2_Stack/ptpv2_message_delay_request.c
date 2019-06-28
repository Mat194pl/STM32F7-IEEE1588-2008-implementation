/*
 * ptpv2_message_delay_request.c
 *
 *  Created on: 6 lis 2016
 *      Author: Mat
 */

#include "ptpv2_message_delay_request.h"
#include "ptpv2_helper.h"
#include "ptpv2_common.h"

extern volatile uint8_t hasDelayRequest;
extern volatile uint32_t lastDelayRequestHighTimeStamp;
extern volatile uint32_t lastDelayRequestLowTimeStamp;
extern volatile uint32_t lastReceivedHighTimeStamp;
extern volatile uint32_t lastReceivedLowTimeStamp;
extern void ptpv2_master_send_delay_respone(void);


void ptpv2_message_delay_request_inverse_bytes(ptpv2_message_delay_request_t* message);
void ptpv2_message_delay_request_receive_callback(ptpv2_message_delay_request_t* message);

ptpv2_message_descriptor_t PTPV2_MESSAGE_DELAY_REQUEST = {
		.inverse_bytes_func = (void(*)(void*))ptpv2_message_delay_request_inverse_bytes,
		.messageLength = 44,
		.messageType = PTPV2_HEADER_MESSAGE_TYPE_DELAY_REQUEST,
		.receive_callback_func = (void(*)(void*))ptpv2_message_delay_request_receive_callback
};

void ptpv2_message_delay_request_inverse_bytes(ptpv2_message_delay_request_t* message)
{
	ptpv2_helper_inverse_bytes((uint8_t*)&message->originTimestamp, 10);
}

void ptpv2_message_delay_request_receive_callback(ptpv2_message_delay_request_t* message)
{
	lastDelayRequestLowTimeStamp = lastReceivedLowTimeStamp;
	lastDelayRequestHighTimeStamp = lastReceivedHighTimeStamp;

	ptpv2_master_send_delay_respone();

	hasDelayRequest = 1;
}
