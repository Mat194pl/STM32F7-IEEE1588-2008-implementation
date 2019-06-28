/*
 * ptpv2_ethernet_frame.h
 *
 *  Created on: 22 paü 2016
 *      Author: Mat
 */

#ifndef PTPV2_ETHERNET_FRAME_H_
#define PTPV2_ETHERNET_FRAME_H_

typedef struct __attribute__((__packed__)) {
	char source_MAC[6];
	char destination_MAC[6];
	unsigned short frame_type;
	char* ptpv2_message;
} ptpv2_ethernet_frame_t;

#endif /* PTPV2_ETHERNET_FRAME_H_ */
