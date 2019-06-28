/*
 * ptpv2_core.c
 *
 *  Created on: 22 paü 2016
 *      Author: Mat
 */

#include "ptpv2_core.h"
#include <stdlib.h>
#include <string.h>
#include "ethernetif.h"
#include "ptpv2_header.h"
#include "ptpv2_message_sync.h"
#include "ptpv2_message_follow_up.h"
#include "ptpv2_message_delay_request.h"
#include "ptpv2_message_delay_response.h"
#include "ptpv2_message_signaling.h"
#include "ptpv2_helper.h"

volatile uint8_t isMaster = 0;

extern uint8_t Device_MAC[6];
extern uint8_t Destination_MAC[];

extern ptpv2_message_descriptor_t PTPV2_MESSAGE_SYNC;
extern ptpv2_message_descriptor_t PTPV2_MESSAGE_FOLLOW_UP;
extern ptpv2_message_descriptor_t PTPV2_MESSAGE_DELAY_REQUEST;
extern ptpv2_message_descriptor_t PTPV2_MESSAGE_DELAY_RESPONSE;
extern ptpv2_message_descriptor_t PTPV2_MESSAGE_SIGNALING;
volatile uint32_t lastSyncHighTimeStamp;
volatile uint32_t lastSyncLowTimeStamp;

volatile uint32_t lastFollowUpHighTimeStamp;
volatile uint32_t lastFollowUpLowTimeStamp;

volatile uint32_t lastDelayRequestHighTimeStamp;
volatile uint32_t lastDelayRequestLowTimeStamp;

volatile uint32_t lastDelayResponseHighTimeStamp;
volatile uint32_t lastDelayResponseLowTimeStamp;

volatile uint32_t lastReceivedHighTimeStamp;
volatile uint32_t lastReceivedLowTimeStamp;

volatile uint8_t hasFollowUp;
volatile uint8_t hasDelayRequest;
volatile uint8_t hasDiff;
volatile uint8_t isImprovingReady;

volatile ptpv2_time_stamp_t current_diff;
volatile ptpv2_time_stamp_t current_diff_change;

ptpv2_message_descriptor_t* PTPV2_Message_Descriptors_Table[] = {
		&PTPV2_MESSAGE_ANNOUNCE,
		&PTPV2_MESSAGE_SYNC,
		&PTPV2_MESSAGE_FOLLOW_UP,
		&PTPV2_MESSAGE_DELAY_REQUEST,
		&PTPV2_MESSAGE_DELAY_RESPONSE,
		&PTPV2_MESSAGE_SIGNALING
};

#define PTPV2_MEMORY_LEVEL 2

static void standard_algorithm(void);
static void improved_algorithm(void);

static uint8_t ptpv2_find_message_descriptor(ptpv2_message_descriptor_t** message_descriptor, uint8_t message_type);

void ptpv2_create_message(ptpv2_message_descriptor_t* desc, uint8_t* message)
{
	((ptpv2_message_common_header_t*)(message))->descriptor = desc;
	((ptpv2_message_common_header_t*)(message))->header.messageLength = desc->messageLength;
	((ptpv2_message_common_header_t*)(message))->header.messageType_transportSpecific = 0;

	((ptpv2_message_common_header_t*)(message))->header.messageType_transportSpecific = desc->messageType;
	((ptpv2_message_common_header_t*)(message))->header.versionPTP = 2;
	((ptpv2_message_common_header_t*)(message))->header.controlField = 0;

	for (int i = 0; i < 8; i++)
		((ptpv2_message_common_header_t*)(message))->header.correctionField[i] = 0;

	((ptpv2_message_common_header_t*)(message))->header.domainNumber = 0;
	((ptpv2_message_common_header_t*)(message))->header.flags = 0;
	((ptpv2_message_common_header_t*)(message))->header.logMessageInterval = 0;
	((ptpv2_message_common_header_t*)(message))->header.reserved = 0;

	for (int i = 0; i < 4; i++)
		((ptpv2_message_common_header_t*)(message))->header.reserved_2[i] = 0;

	((ptpv2_message_common_header_t*)(message))->header.sequenceID = 0;

	for (int i = 0; i < 10; i++)
		((ptpv2_message_common_header_t*)(message))->header.sourcePortIdentity[i] = 0;
}

void ptpv2_send_message(uint8_t* message)
{
	/*ptpv2_ethernet_frame_t new_frame;
	//ptpv2_header_t* message_header = (ptpv2_header_t*)message;

	memcpy(&new_frame.source_MAC[0], &Device_MAC[0], 6);
	memcpy(&new_frame.destination_MAC[0], &Destination_MAC[0], 6);
	new_frame.frame_type = 0xF788;

	new_frame.ptpv2_message = (char*)message;
	ptpv2_send_frame(&new_frame);*/
}

void ptpv2_send_frame(ptpv2_ethernet_frame_t* frame, uint32_t* highTimeStamp, uint32_t* lowTimeStamp)
{
	ptpv2_message_common_header_t* mess_header = ((ptpv2_message_common_header_t*)(frame->ptpv2_message));
	uint16_t message_size = mess_header->header.messageLength;
	//uint8_t message_type = ((ptpv2_message_common_header_t*)(frame->ptpv2_message))->header.messageType_transportSpecific & 0xF;
	if (message_size)
	{
		uint8_t* frame_block = malloc(message_size + 6 + 6 + 2);
		memset(frame_block, 0, message_size + 6 + 6 + 2);

		memcpy(&frame_block[0], &frame->destination_MAC[0], 6);
		memcpy(&frame_block[6], &frame->source_MAC[0], 6);
		memcpy(&frame_block[12], (const void*)(&frame->frame_type), 2);
		/*ptpv2_message_announce_t* ann = frame->ptpv2_message;
		LCD_UsrLog("Mess2 size: %d\n", ann->header.messageLength);*/


		// TODO: memcpy(&frame_block[14], frame->ptpv2_message, message_size);

		memcpy(&frame_block[14], (frame->ptpv2_message + 4), message_size);
		
		ptpv2_header_inverse_bytes((uint8_t*)(&frame_block[14]));

		ptpv2_message_descriptor_t* mess_descriptor = ((ptpv2_message_common_header_t*)((frame->ptpv2_message)))->descriptor;
		
		mess_descriptor->inverse_bytes_func((uint8_t*)&frame_block[10]);

		low_level_output(frame_block, message_size + 6 + 6 + 2);

		ptpv2_wait_for_tx_complete();
		ptpv2_time_stamp_t tx_time_stamp = ptpv2_get_last_tx_time_stamp();
		if (highTimeStamp)
			*highTimeStamp = tx_time_stamp.seconds;

		if (lowTimeStamp)
			*lowTimeStamp = tx_time_stamp.nanoseconds;

		free(frame_block);
	}
}

void ptpv2_receive_frame(uint8_t* data, uint32_t highTimeStamp, uint32_t lowTimeStamp)
{
	ptpv2_header_inverse_bytes((uint8_t*)(&data[0]));

	ptpv2_header_t* header = (ptpv2_header_t*)data;

	ptpv2_message_descriptor_t* message_descriptor;

	lastReceivedLowTimeStamp = lowTimeStamp;
	lastReceivedHighTimeStamp = highTimeStamp;

	if(ptpv2_find_message_descriptor(&message_descriptor, header->messageType_transportSpecific & 0xF))
	{
		message_descriptor->inverse_bytes_func(data - 4); // 4 bytes offset (for message_descriptor pointer)
		message_descriptor->receive_callback_func(data - 4); // 4 bytes offset (for message_descriptor pointer)
	}
	else
	{
		//LOG("PTPV2 UNKNOWN MESSAGE\n");
	}
}

static uint8_t ptpv2_find_message_descriptor(
		ptpv2_message_descriptor_t** message_descriptor, uint8_t message_type)
{
	for (int i = 0; i < 5; i++)
	{
		if (PTPV2_Message_Descriptors_Table[i]->messageType == message_type)
		{
			*message_descriptor = PTPV2_Message_Descriptors_Table[i];
			return 1;
		}
	}

	return 0;
}

static void ptpv2_slave_send_diff_data(volatile ptpv2_time_stamp_t* differenceTimeStamp)
{
	ptpv2_message_signaling_t diff_signal_message;
	ptpv2_ethernet_frame_t frame;

	memcpy(&frame.source_MAC, &Device_MAC, 6);
	memcpy(&frame.destination_MAC, &Destination_MAC, 6);

	ptpv2_create_message(&PTPV2_MESSAGE_SIGNALING, (uint8_t*)&diff_signal_message);

	memset(&(diff_signal_message.targetPortIdentity), 0, 10);
	memset(&(diff_signal_message.payload), 0, 255);

	memcpy((uint8_t*)(&(diff_signal_message.payload)), ((uint8_t*)differenceTimeStamp), sizeof(ptpv2_time_stamp_t));

	diff_signal_message.payload[0] = differenceTimeStamp->sign;
	diff_signal_message.payload[1] = (uint8_t)(differenceTimeStamp->nanoseconds >> 24);
	diff_signal_message.payload[2] = (uint8_t)(differenceTimeStamp->nanoseconds >> 16);
	diff_signal_message.payload[3] = (uint8_t)(differenceTimeStamp->nanoseconds >> 8);
	diff_signal_message.payload[4] = (uint8_t)(differenceTimeStamp->nanoseconds);
	diff_signal_message.payload[5] = (uint8_t)(differenceTimeStamp->seconds >> 24);
	diff_signal_message.payload[6] = (uint8_t)(differenceTimeStamp->seconds >> 16);
	diff_signal_message.payload[7] = (uint8_t)(differenceTimeStamp->seconds >> 8);
	diff_signal_message.payload[8] = (uint8_t)(differenceTimeStamp->seconds);

	LOG("Test %d %d\n\r", diff_signal_message.payload[1], differenceTimeStamp->nanoseconds);

	frame.frame_type = 0xF788;
	frame.ptpv2_message = (char*)&diff_signal_message;

	ptpv2_send_frame(&frame, 0, 0);
	//LOG("T3: %s%u\.%u\n", delay_request_time_stamp.sign ? "-" : "", delay_request_time_stamp.seconds, delay_request_time_stamp.nanoseconds);
}


void ptpv2_slave_send_delay_request(void)
{
	ptpv2_message_delay_request_t delay_request;
	ptpv2_ethernet_frame_t frame;

	memcpy(&frame.source_MAC, &Device_MAC, 6);
	memcpy(&frame.destination_MAC, &Destination_MAC, 6);

	ptpv2_create_message(&PTPV2_MESSAGE_DELAY_REQUEST, (uint8_t*)&delay_request);

	frame.frame_type = 0xF788;
	frame.ptpv2_message = (char*)&delay_request;

	ptpv2_send_frame(&frame, &lastDelayRequestHighTimeStamp, &lastDelayRequestLowTimeStamp);

	ptpv2_time_stamp_t delay_request_time_stamp;
	delay_request_time_stamp.sign = 0;
	delay_request_time_stamp.nanoseconds = lastDelayRequestLowTimeStamp;
	delay_request_time_stamp.seconds = lastDelayRequestHighTimeStamp;

	//LOG("T3: %s%u\.%u\n", delay_request_time_stamp.sign ? "-" : "", delay_request_time_stamp.seconds, delay_request_time_stamp.nanoseconds);
}

static void ptpv2_master_send_sync(void)
{
	ptpv2_message_sync_t sync;
	ptpv2_ethernet_frame_t frame;
	memcpy(&frame.source_MAC, &Device_MAC, 6);
	memcpy(&frame.destination_MAC, &Destination_MAC, 6);

	ptpv2_create_message(&PTPV2_MESSAGE_SYNC, (uint8_t*)&sync);

	frame.frame_type = 0xF788;
	frame.ptpv2_message = (char*)&sync;

	ptpv2_send_frame(&frame, &lastSyncHighTimeStamp, &lastSyncLowTimeStamp);
	ptpv2_time_stamp_t sync_time_stamp;
	sync_time_stamp.nanoseconds = lastSyncLowTimeStamp;
	sync_time_stamp.seconds = lastSyncHighTimeStamp;
	sync_time_stamp.sign = 0;

	LOG("T1: %s%u\.%09u\n", sync_time_stamp.sign ? "-" : "", sync_time_stamp.seconds, sync_time_stamp.nanoseconds);

}

static void ptpv2_master_send_follow_up(void)
{
	ptpv2_message_follow_up_t follow_up;
	ptpv2_ethernet_frame_t frame;
	memcpy(&frame.source_MAC, &Device_MAC, 6);
	memcpy(&frame.destination_MAC, &Destination_MAC, 6);

	ptpv2_create_message(&PTPV2_MESSAGE_FOLLOW_UP, (uint8_t*)&follow_up);

	memset(follow_up.preciseOriginTimestamp, 0, 10);

	for (int i = 0; i < 4; i++)
	{
		follow_up.preciseOriginTimestamp[i] = ((lastSyncLowTimeStamp) >> (i * 8)) & 0xFF;
	}
	for (int i = 4; i < 8; i++)
	{
		follow_up.preciseOriginTimestamp[i] = ((lastSyncHighTimeStamp) >> ((i - 4) * 8))  & 0xFF;
	}

	follow_up.preciseOriginTimestamp[8] = 0;
	follow_up.preciseOriginTimestamp[9] = 0;

	frame.frame_type = 0xF788;
	frame.ptpv2_message = (char*)&follow_up;

	ptpv2_send_frame(&frame, 0, 0);
}

void ptpv2_master_send_delay_respone(void)
{
	ptpv2_message_delay_response_t delay_response;
	ptpv2_ethernet_frame_t frame;
	memcpy(&frame.source_MAC, &Device_MAC, 6);
	memcpy(&frame.destination_MAC, &Destination_MAC, 6);

	ptpv2_create_message(&PTPV2_MESSAGE_DELAY_RESPONSE, (uint8_t*)&delay_response);

	memset(delay_response.receiveTimestamp, 0, 10);

	for (int i = 0; i < 4; i++)
	{
		delay_response.receiveTimestamp[i] = ((lastDelayRequestLowTimeStamp) >> (i * 8)) & 0xFF;
	}
	for (int i = 4; i < 8; i++)
	{
		delay_response.receiveTimestamp[i] = ((lastDelayRequestHighTimeStamp) >> ((i - 4) * 8))  & 0xFF;
	}

	delay_response.receiveTimestamp[8] = 0;
	delay_response.receiveTimestamp[9] = 0;

	frame.frame_type = 0xF788;
	frame.ptpv2_message = (char*)&delay_response;

	ptpv2_send_frame(&frame, 0, 0);
}

static void master_thread(uint32_t lasted_miliseconds)
{
	if ((lasted_miliseconds % 200) == 0) // Send sync sequence every 5 sec
	{
		ptpv2_master_send_sync();
		osDelay(50);
		ptpv2_master_send_follow_up();
	}
	/*if (hasDelayRequest)
	{
		hasDelayRequest = 0;
		ptpv2_master_send_delay_respone();
	}*/
}

static void slave_thread(uint32_t lasted_miliseconds)
{
	if ((lasted_miliseconds % 40) == 0)
	{


		if (hasDiff)
			ptpv2_slave_send_diff_data(&current_diff);


		if (current_diff.nanoseconds < 1000 && current_diff.seconds == 0)
		{

			/*if ((lasted_miliseconds % 100) == 0)
			{
				improved_algorithm();
			}*/
			standard_algorithm();
			//hasDiff = 0;
		}
		else
		{
			isImprovingReady = 0;
			standard_algorithm();
		}

	}
	/*if (hasFollowUp)
	{
		hasFollowUp = 0;
		ptpv2_slave_send_delay_request();
	}*/

}

void ptpv2_init(void)
{

}

void ptpv2_thread(uint32_t period)
{
	static uint32_t lasted_miliseconds;
	lasted_miliseconds += period;
	if ((lasted_miliseconds % 1000) == 0)
	{
		ptpv2_time_stamp_t curr_time = ptpv2_get_current_time();
		if (curr_time.sign != 2)
		{
			LOG("Current time: %s%u\.%u %d\n", curr_time.sign ? "-" : "", curr_time.seconds, curr_time.nanoseconds, lasted_miliseconds);
		}
	}
	if (isMaster)
	{
		master_thread(lasted_miliseconds);
	}
	else
	{
		slave_thread(lasted_miliseconds);
	}
}

static void standard_algorithm(void)
{
	if (hasDiff)
	{
		hasDiff = 0;
		LOG("Correction!\n");
		ptpv2_correct_system_clock(current_diff);
	}
}

static void improved_algorithm(void)
{
	static ptpv2_time_stamp_t last_diff;
	ptpv2_time_stamp_t change_speed;
	if (hasDiff)
	{
		hasDiff = 0;
		ptpv2_time_stamp_t ll;

		change_speed = ptpv2_time_stamp_sub(current_diff, last_diff);

		ll.sign = current_diff.sign;
		ll.nanoseconds = current_diff.nanoseconds / 10;
		change_speed.nanoseconds /= 4;
		current_diff_change = ptpv2_time_stamp_add(change_speed, current_diff_change);


		last_diff = current_diff;




	}	//

	/*ptpv2_time_stamp_t test;
	test.sign = 1;
	test.seconds = 0;
	test.nanoseconds = 45;
	ptpv2_correct_system_clock(test);*/

}
