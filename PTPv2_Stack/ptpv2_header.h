/*
 * ptpv2_header.h
 *
 *  Created on: 22 paü 2016
 *      Author: Mat
 */

#ifndef PTPV2_HEADER_H_
#define PTPV2_HEADER_H_

#include "ptpv2_common.h"

#if defined ( __GNUC__   )
typedef struct __attribute__((__packed__)) {
#else
typedef __packed struct {
#endif
	uint8_t messageType_transportSpecific;
	uint8_t versionPTP;
	uint16_t messageLength;
	uint8_t domainNumber;
	uint8_t reserved;
	uint16_t flags;
	uint8_t correctionField[8];
	uint8_t reserved_2[4];
	uint8_t sourcePortIdentity[10];
	uint16_t sequenceID;
	uint8_t controlField;
	uint8_t logMessageInterval;
} ptpv2_header_t;

#if defined ( __GNUC__   )
typedef struct __attribute__((__packed__)) {
#else
typedef __packed struct {
#endif
	ptpv2_message_descriptor_t* descriptor;
	ptpv2_header_t header;
} ptpv2_message_common_header_t;

void ptpv2_header_inverse_bytes(uint8_t* header);

#endif /* PTPV2_HEADER_H_ */
