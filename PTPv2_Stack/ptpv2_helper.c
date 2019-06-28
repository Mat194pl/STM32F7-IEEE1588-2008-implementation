/*
 * ptpv2_helper.c
 *
 *  Created on: 5 lis 2016
 *      Author: Mat
 */

#include "ptpv2_helper.h"

void ptpv2_helper_inverse_bytes(uint8_t* data, uint32_t length)
{
	uint8_t temp;

	for (int i = 0, j = length - 1; i < j; i++, j--)
	{
		temp = data[i];
		data[i] = data[j];
		data[j] = temp;
	}
}

ptpv2_time_stamp_t ptpv2_time_stamp_add(ptpv2_time_stamp_t first, ptpv2_time_stamp_t second)
{
	ptpv2_time_stamp_t result;
	result.seconds = 0;
	result.nanoseconds = 0;
	result.sign = 0;
	if ((first.sign == 0 && second.sign == 0) || (first.sign == 1 && second.sign == 1))
	{
		result.seconds = first.seconds + second.seconds;
		result.nanoseconds = first.nanoseconds + second.nanoseconds;
		while (result.nanoseconds > 999999999)
		{
			result.nanoseconds -= 1000000000;
			result.seconds++;
		}
		result.sign = first.sign;
	}
	else
	{
		ptpv2_time_stamp_t positive;
		ptpv2_time_stamp_t negative;

		if (first.sign == 1)
		{
			positive = second;
			negative = first;
		}
		else
		{
			positive = first;
			negative = second;
		}

		if (positive.seconds > negative.seconds)
		{
			result.seconds = positive.seconds - negative.seconds;
			if (positive.nanoseconds >= negative.nanoseconds)
			{
				result.nanoseconds = positive.nanoseconds - negative.nanoseconds;
			}
			else
			{
				result.seconds--;
				result.nanoseconds = 1000000000 - (negative.nanoseconds - positive.nanoseconds);
			}
			result.sign = 0;
		}
		else if (positive.seconds == negative.seconds)
		{
			result.seconds = 0;
			if (positive.nanoseconds >= negative.nanoseconds)
			{
				result.nanoseconds = positive.nanoseconds - negative.nanoseconds;
			}
			else
			{
				result.nanoseconds = (negative.nanoseconds - positive.nanoseconds);
				result.sign = 1;
			}
		}
		else
		{
			result.seconds = negative.seconds - positive.seconds;
			if (positive.nanoseconds >= negative.nanoseconds)
			{
				result.nanoseconds = 1000000000 - (positive.nanoseconds - negative.nanoseconds);
				result.seconds--;
			}
			else
			{
				result.nanoseconds = negative.nanoseconds - positive.nanoseconds;
			}
			result.sign = 1;
		}
	}

	return result;
}

ptpv2_time_stamp_t ptpv2_time_stamp_sub(ptpv2_time_stamp_t first, ptpv2_time_stamp_t second)
{
	ptpv2_time_stamp_t result;
	if (second.sign) second.sign = 0;
	else second.sign = 1;
	result = ptpv2_time_stamp_add(first, second);
	return result;
}

__attribute__((weak)) ptpv2_time_stamp_t ptpv2_get_current_time()
{
	ptpv2_time_stamp_t null;
	null.nanoseconds = 0;
	null.seconds = 0;
	null.sign = 2;
	return null;
}

