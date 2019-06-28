/*
 * ptpv2_core.h
 *
 *  Created on: 22 paü 2016
 *      Author: Mat
 */

#ifndef PTPV2_CORE_H_
#define PTPV2_CORE_H_


#include "ptpv2_conf.h"

#include "ptpv2_ethernet_frame.h"
#include "ptpv2_common.h"

extern ptpv2_message_descriptor_t PTPV2_MESSAGE_ANNOUNCE;

void ptpv2_init(void);
void ptpv2_create_message(ptpv2_message_descriptor_t* desc, uint8_t* message);
void ptpv2_send_message(uint8_t* message);
void ptpv2_send_frame(ptpv2_ethernet_frame_t* frame, uint32_t* highTimeStamp, uint32_t* lowTimeStamp);
void ptpv2_receive_frame(uint8_t* data, uint32_t highTimeStamp, uint32_t lowTimeStamp);
void ptpv2_thread(uint32_t period);

#endif /* PTPV2_CORE_H_ */
