/*
 * ptpv2_message_announce.c
 *
 *  Created on: 5 lis 2016
 *      Author: Mat
 */

#include "ptpv2_message_announce.h"
#include "ptpv2_helper.h"
#include "ptpv2_common.h"

void ptpv2_message_announce_inverse_bytes(ptpv2_message_announce_t* message);
void ptpv2_message_announce_receive_callback(ptpv2_message_announce_t* message);

ptpv2_message_descriptor_t PTPV2_MESSAGE_ANNOUNCE = {
		.inverse_bytes_func = (void(*)(void*))ptpv2_message_announce_inverse_bytes,
		.messageLength = 64,
		.messageType = PTPV2_HEADER_MESSAGE_TYPE_ANNOUNCE,
		.receive_callback_func = (void(*)(void*))ptpv2_message_announce_receive_callback
};

void ptpv2_message_announce_inverse_bytes(ptpv2_message_announce_t* message)
{
	ptpv2_helper_inverse_bytes((uint8_t*)&message->currentUtcOffset, 2);
	ptpv2_helper_inverse_bytes((uint8_t*)&message->grandmasterClockQuality, 4);
	ptpv2_helper_inverse_bytes((uint8_t*)&message->grandmasterIdentity, 8);
	ptpv2_helper_inverse_bytes((uint8_t*)&message->originTimestamp, 10);
	ptpv2_helper_inverse_bytes((uint8_t*)&message->stepsRemoved, 2);
}
void ptpv2_message_announce_receive_callback(ptpv2_message_announce_t* message)
{
	#ifdef LOG
		LOG("Received announce message\n");
	#endif
	//LCD_UsrLog("Received announce message\n");
}
