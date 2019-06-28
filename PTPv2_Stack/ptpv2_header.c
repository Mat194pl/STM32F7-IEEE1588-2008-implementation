/*
 * ptpv2_header.c
 *
 *  Created on: 23 paŸ 2016
 *      Author: Mat
 */

#include "ptpv2_header.h"

void ptpv2_header_inverse_bytes(uint8_t* header)
{
	uint8_t temp;

	// Inverse messageLength
	temp = header[2];
	header[2] = header[3];
	header[3] = temp;

	// Inverse flags
	temp = header[6];
	header[6] = header[7];
	header[7] = temp;

	// Inverse correctionField
	for(int i = 0, j = 7; i < 4; i++, j--)
	{
		temp = header[8 + i];
		header[8 + i] = header[8 + j];
		header[8 + j] = temp;
	}

	// Inverse sourrcePortIdentity
	for(int i = 0, j = 9; i < 5; i++, j--)
	{
		temp = header[20 + i];
		header[20 + i] = header[20 + j];
		header[20 + j] = temp;
	}

	// Inverse sequenceID
	temp = header[30];
	header[30] = header[31];
	header[31] = temp;
}
