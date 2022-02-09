/*! ----------------------------------------------------------------------------
*  @file    main_nRF_responder.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           This is a simple code example which acts as the responder in a SS TWR distance measurement exchange. 
*           This application waits for a "poll" message (recording the RX time-stamp of the poll) expected from 
*           the "SS TWR initiator" example code (companion to this application), and
*           then sends a "response" message recording its TX time-stamp, 
*
* @attention
*
* Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/

#include "sdk_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf.h"
#include "app_error.h"

#include <stdio.h>
#include <string.h>

#include "UART.h"

#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"


// Defines ---------------------------------------------


//-----------------dw1000----------------------------

//static dwt_config_t config = {
//  1, /* Channel number. */
//  DWT_PRF_16M, /* Pulse repetition frequency. */
//  DWT_PLEN_2048, /* Preamble length. Used in TX only. */
//  DWT_PAC32, /* Preamble acquisition chunk size. Used in RX only. */
//  1, /* TX preamble code. Used in TX only. */
//  1, /* RX preamble code. Used in RX only. */
//  0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
//  DWT_BR_110K, /* Data rate. */
//  DWT_PHRMODE_STD, /* PHY header mode. */
//  (2048 + 1 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//};

static dwt_config_t config = {
  5,                /* Channel number. */
  DWT_PRF_64M,      /* Pulse repetition frequency. */
  DWT_PLEN_128,     /* Preamble length. Used in TX only. */
  DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
  10,               /* TX preamble code. Used in TX only. */
  10,               /* RX preamble code. Used in RX only. */
  0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
  DWT_BR_6M8,       /* Data rate. */
  DWT_PHRMODE_STD,  /* PHY header mode. */
  (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static void dw1000_init()
{
  /* Setup DW1000 IRQ pin */
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 		//irq

  /* Reset DW1000 */
  reset_DW1000(); 

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();			

  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
    //Init of DW1000 Failed
    while (1)
    {};
  }

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();  

  /* Configure DW1000. */
  dwt_configure(&config);

  /* Set preamble timeout for expected frames.  */
  //dwt_setpreambledetecttimeout(PRE_TIMEOUT);  
}
//--------------dw1000---end---------------

static void print_rtos_status()
{
	printf("%07lu Free heap: %u\n",xTaskGetTickCount(), xPortGetFreeHeapSize());
	
        static TaskStatus_t pxTaskStatusArray[32];
	UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();

	if (uxArraySize < 32)
	{
		uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);

		printf("Name         \tNr\tWaterMark\n");

		for (uint8_t i = 0; i < uxArraySize; i++)
		{
			printf("%-13s\t%u\t%u\n",
				pxTaskStatusArray[i].pcTaskName,
				pxTaskStatusArray[i].xTaskNumber,
				pxTaskStatusArray[i].usStackHighWaterMark);
		}
	}
}

static void rtos_status_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  while (true)
  {
    vTaskDelay(1000);
    print_rtos_status();
  }
}

#define RX_BUF_LEN 24
static void dwt_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  uint32 status_reg = 0;
  static uint8 rx_buffer[RX_BUF_LEN];

  dwt_setrxtimeout(0);

  for(;;){
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {};

      #if 0	  // Include to determine the type of timeout if required.
      int temp = 0;
      // (frame wait timeout and preamble detect timeout)
      if(status_reg & SYS_STATUS_RXRFTO )
      temp =1;
      else if(status_reg & SYS_STATUS_RXPTO )
      temp =2;
      #endif

    if (status_reg & SYS_STATUS_RXFCG)
    {
      uint32 frame_len;

      /* Clear good RX frame event in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

      /* A frame has been received, read it into the local buffer. */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
      if (frame_len <= RX_BUFFER_LEN)
      {
        dwt_readrxdata(rx_buffer, frame_len, 0);
        printf("%02X %02X\n",rx_buffer[2],rx_buffer[6]);

        //rx_buffer[2] = frame_seq_nb++;
        //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        //dwt_writetxdata(frame_len, rx_buffer, 0); /* Zero offset in TX buffer. */
        //dwt_writetxfctrl(frame_len, 0, 0); /* Zero offset in TX buffer, ranging. */

        //dwt_starttx(DWT_START_TX_IMMEDIATE);

      }
    }
    else
    {
      /* Clear RX error events in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

      /* Reset RX to properly reinitialise LDE operation. */
      dwt_rxreset();
    }

    vTaskDelay(1);
  }
}

int main(void)
 {
    //UNUSED_VARIABLE(xTaskCreate(rtos_status_task_function, "RTOS_STATUS", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL));
    UNUSED_VARIABLE(xTaskCreate(dwt_task_function, "DWT_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL)); 

    boUART_Init();
    dw1000_init();

    printf("--- START --- \r\n");

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();	
    
    for(;;);
}