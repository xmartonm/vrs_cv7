/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

#define BUFF_SZ		40

/* Function processing DMA Rx data. Counts how many capital and small letters are in sentence.
 * Result is supposed to be stored in global variable of type "letter_count_" that is defined in "main.h"
 *
 * @param1 - received sign
 */
void proccesDmaData(uint8_t sign);


/* Space for your global variables. */

uint16_t dma_buff_pos = 0;


int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  /* Space for your local variables, callback registration ...*/

  USART2_RegisterCallback(proccesDmaData);

  while (1)
  {
	  /* Periodic transmission of information about DMA Rx buffer state.
	   * Transmission frequency - 5Hz.
	   * Message format - "Buffer capacity: %d bytes, occupied memory: %d bytes, load [in %]: %f%"
	   * Example message (what I wish to see in terminal) - Buffer capacity: 1000 bytes, occupied memory: 231 bytes, load [in %]: 23.1%
	   */

  	  uint8_t buff[30];
  	  memset(buff, 0, 30);
  	  uint8_t sz = sprintf(buff, "DMA: %d\n", dma_buff_pos);
  	  USART2_PutBuffer(buff, sz);
  	  LL_mDelay(500);
  }
  /* USER CODE END 3 */
}


void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
}

/*
 * Implementation of function processing data received via USART.
 */

typedef enum {
	RECEIVING,
	IGNORING
}STATES;

void processCmd(uint8_t *buff, uint16_t len) {
	uint16_t cnt_lc = 0, cnt_uc = 0;
	for(uint16_t i = 0; i < len; ++i) {
		uint8_t c = buff[i];
		if(c <= 'z' && c >= 'a') {
			++cnt_lc;
		}
		else if(c <= 'Z' && c >= 'A') {
			++cnt_uc;
		}
	}

	uint8_t str_buff[30];
	memset(str_buff, 0, 30);
	uint8_t sz = sprintf(str_buff, "\n\tLC: %d, UC: %d\n", cnt_lc, cnt_uc);
	USART2_PutBuffer(str_buff, sz);
}

void proccesDmaData(uint8_t sign)
{
	/* Process received data */
	static uint8_t buffer[BUFF_SZ];
	static uint16_t ind = 0;
	static STATES state_machine = IGNORING;

	if(sign == '#' && state_machine != RECEIVING) {
		state_machine = RECEIVING;
		ind = 0;
	}
	if(state_machine == RECEIVING) {
		buffer[ind++] = sign;
	}
	if(sign == '$' || ind >= 35) {
		state_machine = IGNORING;
		processCmd(buffer, ind);
	}
}


void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{ 

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
