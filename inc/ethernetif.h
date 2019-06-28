#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include "cmsis_os.h"

uint8_t ethernetif_init();
uint8_t low_level_output(uint8_t* data, uint32_t dataLength);

void ETHERNET_IRQHandler(void);

#endif
