/*
 * ptpv2_message_path_delay_request.c
 *
 *  Created on: 30 lis 2016
 *      Author: Mat
 */

#include "ptpv2_message_path_delay_request.h"
#include "ptpv2_helper.h"

void ptpv2_message_path_delay_request_inverse_bytes(ptpv2_message_path_delay_request_t* message);
void ptpv2_message_path_delay_request_receive_callback(ptpv2_message_path_delay_request_t* message);

ptpv2_message_descriptor_t PTPV2_MESSAGE_PATH_DELAY_REQUEST = {
		.inverse_bytes_func = (const void(*)(void*))ptpv2_message_path_delay_request_inverse_bytes,
		.messageLength = 44,
		.messageType = PTPV2_HEADER_MESSAGE_TYPE_PATH_DELAY_REQUEST,
		.receive_callback_func = (const void(*)(void*))ptpv2_message_path_delay_request_receive_callback
};


void ptpv2_message_path_delay_request_inverse_bytes(ptpv2_message_path_delay_request_t* message)
{
	ptpv2_helper_inverse_bytes((uint8_t*)&message->OriginTimestamp, 10);
}

void ptpv2_message_path_delay_request_receive_callback(ptpv2_message_path_delay_request_t* message)
{
}

