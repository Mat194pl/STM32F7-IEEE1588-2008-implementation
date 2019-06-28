/*
 * ptpv2_message_signaling.c
 *
 *  Created on: 19 lut 2017
 *      Author: Mat
 */

#include "ptpv2_message_signaling.h"
#include "ptpv2_helper.h"
#include "ptpv2_common.h"

void ptpv2_message_signaling_inverse_bytes(ptpv2_message_signaling_t* message);
void ptpv2_message_signaling_receive_callback(ptpv2_message_signaling_t* message);

ptpv2_message_descriptor_t PTPV2_MESSAGE_SIGNALING = {
		.inverse_bytes_func = (void(*)(void*))ptpv2_message_signaling_inverse_bytes,
		.messageLength = 44 + PTPV2_MESSAGE_SIGNALING_PAYLOAD_MAX_LENGTH,
		.messageType = PTPV2_HEADER_MESSAGE_TYPE_SIGNALING,
		.receive_callback_func = (void(*)(void*))ptpv2_message_signaling_receive_callback
};

void ptpv2_message_signaling_inverse_bytes(ptpv2_message_signaling_t* message)
{
	ptpv2_helper_inverse_bytes((uint8_t*)&message->targetPortIdentity, 10);
}

void ptpv2_message_signaling_receive_callback(ptpv2_message_signaling_t* message)
{

}
