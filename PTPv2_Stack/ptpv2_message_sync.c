/*
 * ptpv2_messsage_sync.c
 *
 *  Created on: 2 gru 2016
 *      Author: Mat
 */

#include "ptpv2_message_sync.h"
#include "ptpv2_helper.h"
#include "ptpv2_common.h"

extern volatile uint32_t lastSyncHighTimeStamp;
extern volatile uint32_t lastSyncLowTimeStamp;
extern volatile uint32_t lastReceivedHighTimeStamp;
extern volatile uint32_t lastReceivedLowTimeStamp;

void ptpv2_message_sync_inverse_bytes(ptpv2_message_sync_t* message);
void ptpv2_message_sync_receive_callback(ptpv2_message_sync_t* message);

ptpv2_message_descriptor_t PTPV2_MESSAGE_SYNC = {
		.inverse_bytes_func = (void(*)(void*))ptpv2_message_sync_inverse_bytes,
		.messageLength = 44,
		.messageType = PTPV2_HEADER_MESSAGE_TYPE_SYNC,
		.receive_callback_func = (void(*)(void*))ptpv2_message_sync_receive_callback
};

void ptpv2_message_sync_inverse_bytes(ptpv2_message_sync_t* message)
{
	ptpv2_helper_inverse_bytes((uint8_t*)&message->originTimestamp, 10);
}

void ptpv2_message_sync_receive_callback(ptpv2_message_sync_t* message)
{
	lastSyncHighTimeStamp = lastReceivedHighTimeStamp;
	lastSyncLowTimeStamp = lastReceivedLowTimeStamp;

	ptpv2_time_stamp_t sync_time_stamp;
	sync_time_stamp.sign = 0;
	sync_time_stamp.nanoseconds = lastReceivedLowTimeStamp;
	sync_time_stamp.seconds = lastReceivedHighTimeStamp;

	//LOG("T2: %s%u\.%09u\n", sync_time_stamp.sign ? "-" : "", sync_time_stamp.seconds, sync_time_stamp.nanoseconds);
}
