#include "sdk_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>
#include <queue.h>

#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
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

/*!
* @brief Configure an IO pin as a positive edge triggered interrupt source.
*/
static void dw1000_int_init(nrf_drv_gpiote_evt_handler_t evt_handler)
{
  ret_code_t err_code;

  if (!nrf_drv_gpiote_is_init())
  {
    nrf_drv_gpiote_init();
  }

  // input pin, +ve edge interrupt, no pull-up
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_NOPULL;

  // Link this pin interrupt source to its interrupt handler
  err_code = nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, evt_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);
}

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


static volatile uint32_t interrupt_count;
static volatile uint32_t rx_count;
static volatile uint32_t rxgood_count;
static volatile uint32_t dw1000_irq_semaphore_timeout;
static void rtos_status_task_function (void * pvParameter)
{
  for(;;)
  {
    vTaskDelay(1000);
    print_rtos_status();
    printf("IntCnt %u RxCnt %u RxGood: %u Rate %u SemTo %u\n",interrupt_count, rx_count, rxgood_count, rxgood_count*100/(rx_count==0?1:rx_count), dw1000_irq_semaphore_timeout);
  }
}

SemaphoreHandle_t dw1000_xIrqSemaphore;

#define RX_BUF_LEN 24
static void dwt_task_function (void * pvParameter)
{
  static uint8 rx_buffer[RX_BUF_LEN];

  dwt_setrxtimeout(0);
  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

  for(;;)
  {
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    if (xSemaphoreTake(dw1000_xIrqSemaphore, pdMS_TO_TICKS(3000)) != pdTRUE)
    {
       // TODO: Only debug
        dw1000_irq_semaphore_timeout++;
    }

    rx_count++;

    uint32 status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    // NOTE: 2022.02.09 VV - No need to pool, just a read after either Interrupt or Semaphore Timeout
    /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
    //while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    //{};

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
        rxgood_count++;
        //printf("%02X %02X\n",rx_buffer[2],rx_buffer[6]);

        //rx_buffer[2] = frame_seq_nb++;
        //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        //dwt_writetxdata(frame_len, rx_buffer, 0); /* Zero offset in TX buffer. */
        //dwt_writetxfctrl(frame_len, 0, 0); /* Zero offset in TX buffer, ranging. */

        //dwt_starttx(DWT_START_TX_IMMEDIATE);

      }
    }
    else
    {
      // NOTE: 2022.02.09 VV - This commented section is simply not enogh to do the reset, using the code below is from the Deca Int handler routine
      ///* Clear RX error events in the DW1000 status register. */
      //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      ///* Reset RX to properly reinitialise LDE operation. */
      //dwt_rxreset();
      
      // NOTE: 2022.02.09 VV - NOTE 1 - Also from the Deca Int handler - Good RX Frame part
      // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
      // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
      // implementation works only for IEEE802.15.4-2011 compliant frames).
      // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
      // NOTE: 2022.02.09 VV - NOTE 2 - Also from the Deca Int handler - TX part
      // In the case where this TXFRS interrupt is due to the automatic transmission of an ACK solicited by a response (with ACK request bit set)
      // that we receive through using wait4resp to a previous TX (and assuming that the IRQ processing of that TX has already been handled), then
      // we need to handle the IC issue which turns on the RX again in this situation (i.e. because it is wrongly applying the wait4resp after the
      // ACK TX).
      // See section "Transmit and automatically wait for response" in DW1000 User Manual
      
      // Clear All Status bits
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_MASK_32);

      // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
      // the next good frame's timestamp is computed correctly.
      // See section "RX Message timestamp" in DW1000 User Manual.
      dwt_forcetrxoff();
      dwt_rxreset();
    }
  }
}

static void dwt_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  interrupt_count++;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(dw1000_xIrqSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int main(void)
{
  xTaskCreate(rtos_status_task_function, "RTOS_STATUS", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL);
  xTaskCreate(dwt_task_function, "DWT_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL); 
  dw1000_xIrqSemaphore = xSemaphoreCreateBinary();

  boUART_Init();
  dw1000_init();

  dw1000_int_init(dwt_int_handler);

  printf("--- START --- \r\n");

  /* Start FreeRTOS scheduler. */
  vTaskStartScheduler();	

  for(;;);
}