/*
 * ptpv2_message_delay_response.c
 *
 *  Created on: 6 lis 2016
 *      Author: Mat
 */

#include "ptpv2_message_delay_response.h"
#include "ptpv2_helper.h"

extern volatile uint32_t lastDelayResponseHighTimeStamp;
extern volatile uint32_t lastDelayResponseLowTimeStamp;
extern volatile uint32_t lastFollowUpHighTimeStamp;
extern volatile uint32_t lastFollowUpLowTimeStamp;
extern volatile uint32_t lastSyncHighTimeStamp;
extern volatile uint32_t lastSyncLowTimeStamp;
extern volatile uint32_t lastDelayRequestHighTimeStamp;
extern volatile uint32_t lastDelayRequestLowTimeStamp;
extern volatile ptpv2_time_stamp_t current_diff;
extern volatile uint8_t hasDiff;

void ptpv2_message_delay_response_inverse_bytes(ptpv2_message_delay_response_t* message);
void ptpv2_message_delay_response_receive_callback(ptpv2_message_delay_response_t* message);

ptpv2_message_descriptor_t PTPV2_MESSAGE_DELAY_RESPONSE = {
		.inverse_bytes_func = (const void(*)(void*))ptpv2_message_delay_response_inverse_bytes,
		.messageLength = 54,
		.messageType = PTPV2_HEADER_MESSAGE_TYPE_DELAY_RESPONSE,
		.receive_callback_func = (const void(*)(void*))ptpv2_message_delay_response_receive_callback
};

void ptpv2_message_delay_response_inverse_bytes(ptpv2_message_delay_response_t* message)
{
	ptpv2_helper_inverse_bytes((uint8_t*)&message->receiveTimestamp, 10);
	ptpv2_helper_inverse_bytes((uint8_t*)&message->requestingPortIdentity, 10);
}


void ptpv2_message_delay_response_receive_callback(ptpv2_message_delay_response_t* message)
{
	memcpy(&lastDelayResponseLowTimeStamp, &(message->receiveTimestamp[0]), 4);
	memcpy(&lastDelayResponseHighTimeStamp, &(message->receiveTimestamp[4]), 4);

	ptpv2_time_stamp_t t1;
	ptpv2_time_stamp_t t2;
	ptpv2_time_stamp_t t3;
	ptpv2_time_stamp_t t4;
	ptpv2_time_stamp_t propagation_time;
	t1.sign = 0;
	t1.nanoseconds = lastFollowUpLowTimeStamp;
	t1.seconds = lastFollowUpHighTimeStamp;

	t2.sign = 0;
	t2.nanoseconds = lastSyncLowTimeStamp;
	t2.seconds = lastSyncHighTimeStamp;

	t3.sign = 0;
	t3.nanoseconds = lastDelayRequestLowTimeStamp;
	t3.seconds = lastDelayRequestHighTimeStamp;

	t4.sign = 0;
	t4.nanoseconds = lastDelayResponseLowTimeStamp;
	t4.seconds = lastDelayResponseHighTimeStamp;

	ptpv2_time_stamp_t diff1 = ptpv2_time_stamp_sub(t2, t1);
	ptpv2_time_stamp_t diff2 = ptpv2_time_stamp_sub(t4, t3);
	ptpv2_time_stamp_t sum = ptpv2_time_stamp_add(diff1, diff2);
	propagation_time = sum;
	propagation_time.nanoseconds /= 2;
	propagation_time.seconds /= 2;

	ptpv2_time_stamp_t diff3 = ptpv2_time_stamp_sub(diff1, diff2);
	diff3.nanoseconds /= 2;

	memset(&current_diff, 0, sizeof(ptpv2_time_stamp_t));

	current_diff = ptpv2_time_stamp_sub(t2, t1);
	//LOG("Diff time: %s%u\.%09u\n", current_diff.sign ? "-" : "", current_diff.seconds, current_diff.nanoseconds);

	current_diff = ptpv2_time_stamp_sub(current_diff, propagation_time);

	current_diff = diff3;


	/*if (hasDiff == 1) LOG("Overflow!\n");*/
	hasDiff = 1;
	/*LOG("t1: %s%u\.%09u\n", t1.sign ? "-" : "", t1.seconds, t1.nanoseconds);
	LOG("t2: %s%u\.%09u\n", t2.sign ? "-" : "", t2.seconds, t2.nanoseconds);
	LOG("t3: %s%u\.%09u\n", t3.sign ? "-" : "", t3.seconds, t3.nanoseconds);
	LOG("t4: %s%u\.%09u\n", t4.sign ? "-" : "", t4.seconds, t4.nanoseconds);*/
/*
	LOG("DIFF1: %s%u, DIFF2: %s%u\n\r", diff1.sign ? "-" : "", diff1.nanoseconds, diff2.sign ? "-" : "", diff2.nanoseconds);*/
	LOG("DIFF: %s%u\n", current_diff.sign ? "-" : "", current_diff.nanoseconds);
	/*LOG("Propagation time: %s%u\.%09u\n", propagation_time.sign ? "-" : "", propagation_time.seconds, propagation_time.nanoseconds);
	LOG("Diff time: %s%u\.%09u\n", current_diff.sign ? "-" : "", current_diff.seconds, current_diff.nanoseconds);*/
}

