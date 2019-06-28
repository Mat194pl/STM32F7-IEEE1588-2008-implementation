/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/ethernetif.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    22-April-2016
  * @brief   This file implements Ethernet network interface drivers for lwIP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "ethernetif.h"
#include <string.h>
#include "lcd_log.h"
#include "stm32746g_discovery.h"
#include "ptpv2_ethernet_frame.h"
#include "ptpv2_helper.h"
#include "stm32f7xx.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT                 ( portMAX_DELAY )
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE            ( 350 )

/* Define those to better describe your network interface. */
#define IFNAME0 's'
#define IFNAME1 't'

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x2000E000
__no_init ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
#pragma location=0x2000E100
__no_init ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
#elif defined ( __CC_ARM   )
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __attribute__((at(0x2000E000)));/* Ethernet Rx MA Descriptor */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __attribute__((at(0x2000E100)));/* Ethernet Tx DMA Descriptor */
#elif defined ( __GNUC__   )
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __attribute__((section(".RxDescripSection")));/* Ethernet Rx MA Descriptor */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __attribute__((section(".TxDescripSection")));/* Ethernet Tx DMA Descriptor */
#endif
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2000E200
__no_init uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; /* Ethernet Receive Buffer */
#pragma location=0x2000FFC4
__no_init uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; /* Ethernet Transmit Buffer */
#elif defined ( __CC_ARM   )
uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((at(0x2000E200)));  /* Ethernet Receive Buffer */
uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]  __attribute__((at(0x2000FFC4))); /* Ethernet Transmit Buffer */
#elif defined ( __GNUC__   )
uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((section(".RxBUF")));/* Ethernet Receive Buffer */
uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__((section(".TxBUF")));/* Ethernet Transmit Buffer */
#endif
/* Semaphore to signal incoming packets */
osSemaphoreId s_xSemaphore = NULL;

extern uint8_t Device_MAC[];

volatile uint8_t transmit_complete;
volatile uint32_t lastTxTimeStampLow;
volatile uint32_t lastTxTimeStampHigh;
volatile uint32_t lastRxTimeStampLow;
volatile uint32_t lastRxTimeStampHigh;
volatile ETH_DMADescTypeDef *LastDmaTxDesc;

/* Global Ethernet handle*/
ETH_HandleTypeDef EthHandle;

/* Private function prototypes -----------------------------------------------*/
static void ethernetif_input( void const * argument );

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
                       Ethernet MSP Routines
*******************************************************************************/
/**
  * @brief  Initializes the ETH MSP.
  * @param  heth: ETH handle
  * @retval None
  */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
/* Ethernet pins configuration ************************************************/
  /*
        RMII_REF_CLK ----------------------> PA1
        RMII_MDIO -------------------------> PA2
        RMII_MDC --------------------------> PC1
        RMII_MII_CRS_DV -------------------> PA7
        RMII_MII_RXD0 ---------------------> PC4
        RMII_MII_RXD1 ---------------------> PC5
        RMII_MII_RXER ---------------------> PG2
        RMII_MII_TX_EN --------------------> PG11
        RMII_MII_TXD0 ---------------------> PG13
        RMII_MII_TXD1 ---------------------> PG14
  */

  /* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Alternate = GPIO_AF11_ETH;
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PC1, PC4 and PC5 */
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PG2, PG11, PG13 and PG14 */
  GPIO_InitStructure.Pin =  GPIO_PIN_2 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_5;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);


  TIM2->OR |= (1 << 10);

  /* Enable the Ethernet global Interrupt */
  HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);

  /* Enable ETHERNET clock  */
  __HAL_RCC_ETH_CLK_ENABLE();
}

/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  heth: ETH handle
  * @retval None
  */




void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
	lastRxTimeStampHigh = EthHandle.Instance->PTPTSHR;
	lastRxTimeStampLow = EthHandle.Instance->PTPTSLR;
	osSemaphoreRelease(s_xSemaphore);
}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
	lastTxTimeStampLow = LastDmaTxDesc->TimeStampLow;
	lastTxTimeStampHigh = LastDmaTxDesc->TimeStampHigh;
	transmit_complete = 1;
}

/**
  * @brief  Ethernet IRQ Handler
  * @param  None
  * @retval None
  */
void ETHERNET_IRQHandler(void)
{

	HAL_ETH_IRQHandler(&EthHandle);
  //LCD_UsrLog("ETH_IRQ\n\r");

}

extern volatile uint8_t isMaster;


void ptpv2_wait_for_tx_complete()
{
	while(!transmit_complete)
	{
		osDelay(1);
	}
	transmit_complete = 0;
}

void ptpv2_correct_system_clock(ptpv2_time_stamp_t offset)
{
	(EthHandle.Instance)->PTPTSLUR = offset.nanoseconds;
	if (offset.sign == 0) (EthHandle.Instance)->PTPTSLUR |= (1 << 31);
	(EthHandle.Instance)->PTPTSHUR = offset.seconds;
	(EthHandle.Instance)->PTPTSCR |= ETH_PTPTSCR_TSSTU;
}

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
  * @brief In this function, the hardware should be initialized.
  * Called from ethernetif_init().
  *
  * @param netif the already initialized lwip network interface structure
  *        for this ethernetif
  */
static void low_level_init()
{

  for (int i = 0; i < 6; i++)
  {
	  Device_MAC[5 - i] = *((uint8_t*)(0x1FF0F420 + i));
  }
  Device_MAC[0] = 0;
  Device_MAC[1] = 0;
  Device_MAC[2] = 0;
  Device_MAC[3] = 0;
  Device_MAC[4] = 0;

  uint8_t macaddress[6]= { Device_MAC[0], Device_MAC[1], Device_MAC[2], Device_MAC[3], Device_MAC[4], Device_MAC[5] };

  if (Device_MAC[5] == 39)
	  isMaster = 1;

  EthHandle.Instance = ETH;
  EthHandle.Init.MACAddr = macaddress;
  EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  EthHandle.Init.Speed = ETH_SPEED_100M;
  EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
  EthHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
  EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  EthHandle.Init.PhyAddress = LAN8742A_PHY_ADDRESS;

  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
  if (HAL_ETH_Init(&EthHandle) == HAL_OK)
  {
    /* Set netif link flag */
    //netif->flags |= NETIF_FLAG_LINK_UP;
  }
  else
  {
	  LCD_ErrLog("ETH_INIT\n");
  }



  /* Initialize Tx Descriptors list: Chain Mode */
  HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);

  /* Initialize Rx Descriptors list: Chain Mode  */
  HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

  /* set netif MAC hardware address length */
  //netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set netif MAC hardware address */
  /*netif->hwaddr[0] =  MAC_ADDR0;
  netif->hwaddr[1] =  MAC_ADDR1;
  netif->hwaddr[2] =  MAC_ADDR2;
  netif->hwaddr[3] =  MAC_ADDR3;
  netif->hwaddr[4] =  MAC_ADDR4;
  netif->hwaddr[5] =  MAC_ADDR5;*/

  /* set netif maximum transfer unit */
  //netif->mtu = 1500;

  /* Accept broadcast address and ARP traffic */
  //netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  /* create a binary semaphore used for informing ethernetif of frame reception */
  osSemaphoreDef(SEM);
  s_xSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1 );

  /* create the task that handles the ETH_MAC */
  osThreadDef(EthIf, ethernetif_input, osPriorityRealtime, 0, INTERFACE_THREAD_STACK_SIZE);
  osThreadCreate (osThread(EthIf), 0);

  for (int i = 0; i < ETH_TXBUFNB; i++)
  {
	  // Transmit Time Stamp Enable
	  DMATxDscrTab[i].Status |= ETH_DMATXDESC_TTSE | ETH_DMATXDESC_IC;
  }

  /*for (int i = 0; i < ETH_RXBUFNB; i++)
  {
	  DMARxDscrTab[i].Status |= ETH_DMARX;
  }*/

  __HAL_RCC_ETHMACPTP_CLK_ENABLE();

  // Enhanced Descriptor Enable
  (EthHandle.Instance)->DMABMR |= ETH_DMABMR_EDE;

  // Time stamp trigger interrupt mask
  (EthHandle.Instance)->MACIMR |= ETH_MACIMR_TSTIM;

  // Time stamp enable
  (EthHandle.Instance)->PTPTSCR = ETH_PTPTSCR_TSE;
  //(EthHandle.Instance)->PTPTSCR |= ETH_PTPTSCR_TSFCU;
  //(EthHandle.Instance)->PTPTSCR |= (1 << 10);

  // Time stamp Sub-seconds rollover
  (EthHandle.Instance)->PTPTSCR |= ETH_PTPTSSR_TSSSR;

  //(EthHandle.Instance)->PTPSSIR = ETH_PTPSSIR_STSSI;
  //(EthHandle.Instance)->PTPTSLUR = 12;
  (EthHandle.Instance)->PTPSSIR = 5;

  //(EthHandle.Instance)->PTPTSAR = 0x3B425ED0;
  while((((EthHandle.Instance)->PTPTSCR) & ETH_PTPTSCR_TSARU ) != 0) {};
  (EthHandle.Instance)->PTPTSCR |= ETH_PTPTSCR_TSSTI /* | ETH_PTPTSSR_TSSPTPOEFE | ETH_PTPTSSR_TSPTPPSV2E*/;

  (EthHandle.Instance)->DMAIER |= ETH_DMAIER_TIE;

  /* Enable MAC and DMA transmission and reception */
  HAL_ETH_Start(&EthHandle);



}

ptpv2_time_stamp_t ptpv2_get_last_tx_time_stamp(void)
{
	ptpv2_time_stamp_t result;
	result.sign = 0;
	result.nanoseconds = lastTxTimeStampLow;
	result.seconds = lastTxTimeStampHigh;
	return result;
}

/**
  * @brief This function should do the actual transmission of the packet. The packet is
  * contained in the pbuf that is passed to the function. This pbuf
  * might be chained.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
  * @return ERR_OK if the packet could be sent
  *         an err_t value if the packet couldn't be sent
  *
  * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
  *       strange results. You might consider waiting for space in the DMA queue
  *       to become available since the stack doesn't retry to send a packet
  *       dropped because of memory failure (except for the TCP timers).
  */
uint8_t low_level_output(uint8_t* data, uint32_t dataLength)
{
  uint8_t *buffer = (uint8_t *)(EthHandle.TxDesc->Buffer1Addr);
  __IO ETH_DMADescTypeDef *DmaTxDesc;
  uint32_t framelength = 0;
  uint32_t bufferoffset = 0;
  uint32_t byteslefttocopy = 0;
  uint32_t payloadoffset = 0;

  DmaTxDesc = EthHandle.TxDesc;
  LastDmaTxDesc = EthHandle.TxDesc;
  bufferoffset = 0;

  /* Is this buffer available? If not, goto error */
  if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
  {
	  /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
	  if ((EthHandle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
	  {
		  /* Clear TUS ETHERNET DMA flag */
		  EthHandle.Instance->DMASR = ETH_DMASR_TUS;

	      /* Resume DMA transmission*/
		  EthHandle.Instance->DMATPDR = 0;
	  }
      return 1;
  }

  /* Get bytes in current lwIP buffer */
  byteslefttocopy = dataLength;
  payloadoffset = 0;

  /* Check if the length of data to copy is bigger than Tx buffer size*/
  while( (byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE )
  {
	  /* Copy data to Tx buffer*/
	  memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)data + payloadoffset), (ETH_TX_BUF_SIZE - bufferoffset) );

      /* Point to next descriptor */
      DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);

      /* Check if the buffer is available */
      if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
      {
        /*errval = ERR_USE;*/
    	// todo
        return 1;
      }

      buffer = (uint8_t *)(DmaTxDesc->Buffer1Addr);

      byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
      payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
      framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
      bufferoffset = 0;
  }

  /* Copy the remaining bytes */
  memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)data + payloadoffset), byteslefttocopy );
  bufferoffset = bufferoffset + byteslefttocopy;
  framelength = framelength + byteslefttocopy;

  /* Prepare transmit descriptors to give to DMA */
  HAL_ETH_TransmitFrame(&EthHandle, framelength);
  //(EthHandle.Instance)->PTPTSCR |= ETH_PTPTSCR_TSSTU;
  //LCD_UsrLog("LOW: %X HIGH: %X\n", EthHandle.Instance->PTPTSLR, EthHandle.Instance->PTPTSHR);

  //LCD_UsrLog("LOWtx: %X HIGHtx: %X\n", DmaTxDesc->TimeStampLow, DmaTxDesc->TimeStampHigh);

  //LCD_UsrLog("PTPTSCR: %d\n", EthHandle.Instance->PTPTSCR);

  return 0;
}

/**
  * @brief Should allocate a pbuf and transfer the bytes of the incoming
  * packet from the interface into the pbuf.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
  */
static void low_level_input()
{
  struct pbuf *p = NULL, *q = NULL;
  uint16_t len = 0;
  uint8_t *buffer;
  __IO ETH_DMADescTypeDef *dmarxdesc;
  uint32_t bufferoffset = 0;
  uint32_t payloadoffset = 0;
  uint32_t byteslefttocopy = 0;
  uint32_t i=0;
  ptpv2_ethernet_frame_t* eth_frame;

  // get received frame
  if(HAL_ETH_GetReceivedFrame_IT(&EthHandle) == HAL_OK)
  {
	  // Obtain the size of the packet and put it into the "len" variable.
	  len = EthHandle.RxFrameInfos.length;
	  buffer = (uint8_t *)EthHandle.RxFrameInfos.buffer;
	  eth_frame = (ptpv2_ethernet_frame_t*)buffer;
	  if (len > 0)
	  {
		// We allocate a pbuf chain of pbufs from the Lwip buffer pool
		//p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

		  if (eth_frame->frame_type == 0xF788)
		  {
			  char* ptp_data;
			  ptp_data = malloc(len - 14); // Remove dest and source address, frame_type
			  memcpy(ptp_data, buffer + 14, len - 14);

			  // TODO: Call ptpv2_stack func
			  ptpv2_receive_frame(ptp_data, lastRxTimeStampHigh, lastRxTimeStampLow);

			  //LCD_UsrLog("PTPV2 Frame LOWtx: %X HIGHtx: %X\n\r", lastTimeStampLow, lastTimeStampHigh);

			  free(ptp_data);
		  }
	  }

	  //if (p != NULL)
	  //{
		dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
		bufferoffset = 0;

		//for(q = p; q != NULL; q = q->next)
		//{
		  byteslefttocopy = len;
		  payloadoffset = 0;

		  // Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size
		  while( (byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE )
		  {
			// Copy data to pbuf
			//memcpy( (uint8_t*)((uint8_t*)q->payload + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));

			// Point to next descriptor
			dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
			buffer = (uint8_t *)(dmarxdesc->Buffer1Addr);

			byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
			payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
			bufferoffset = 0;
		  }

		  // Copy remaining data in pbuf
		  //memcpy( (uint8_t*)((uint8_t*)q->payload + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), byteslefttocopy);
		  bufferoffset = bufferoffset + byteslefttocopy;
		//}
	  //}

	  // Release descriptors to DMA
	  // Point to first descriptor
	  dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
	  // Set Own bit in Rx descriptors: gives the buffers back to DMA
	  for (i=0; i< EthHandle.RxFrameInfos.SegCount; i++)
	  {
		dmarxdesc->Status |= ETH_DMARXDESC_OWN;
		dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
	  }

	  // Clear Segment_Count
	  EthHandle.RxFrameInfos.SegCount =0;

	  // When Rx Buffer unavailable flag is set: clear it and resume reception
	  if ((EthHandle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
	  {
		// Clear RBUS ETHERNET DMA flag
		EthHandle.Instance->DMASR = ETH_DMASR_RBUS;
		// Resume DMA reception
		EthHandle.Instance->DMARPDR = 0;
	  }
  }
}

/**
  * @brief This function is the ethernetif_input task, it is processed when a packet
  * is ready to be read from the interface. It uses the function low_level_input()
  * that should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input( void const * argument )
{
  /*struct pbuf *p;
  struct netif *netif = (struct netif *) argument;
*/
  for( ;; )
  {
    if (osSemaphoreWait( s_xSemaphore, TIME_WAITING_FOR_INPUT)==osOK)
    {
    	low_level_input();
    }
  }
}

/**
  * @brief Should be called at the beginning of the program to set up the
  * network interface. It calls the function low_level_init() to do the
  * actual setup of the hardware.
  *
  * This function should be passed as a parameter to netif_add().
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return ERR_OK if the loopif is initialized
  *         ERR_MEM if private data couldn't be allocated
  *         any other err_t on error
  */
uint8_t ethernetif_init()
{
  /* initialize the hardware */
  low_level_init();

  return 0;
}

ptpv2_time_stamp_t ptpv2_get_current_time()
{
	ptpv2_time_stamp_t curr_time;
	curr_time.sign = 0;
	curr_time.seconds = EthHandle.Instance->PTPTSHR;
	curr_time.nanoseconds = EthHandle.Instance->PTPTSLR;
	return curr_time;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
