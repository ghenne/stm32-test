/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-May-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

uint8_t buffer1[100];
uint8_t buffer2[100];
volatile uint8_t buffer_index;
volatile uint8_t *readbuffer; // pointer to current read buffer
volatile uint8_t *writebuffer; // pointer to current write buffer
volatile uint8_t new_message; // set to one when we have a new line, the reader/consumer must clear it

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void) 
{
	char ch;

	if (USART_GetITStatus(USART1, USART_IT_ORE) == SET) {
		while (1);
	}

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
		ch = (USART_ReceiveData(USART1) & 0x7F);
		// handle overflow by erasing the message
		if (buffer_index >= 90) {
			buffer_index = 0;
			return;
		}
		writebuffer[buffer_index++] = ch;
		if (ch == '\r') {
			//writebuffer[buffer_index++] = '\r';
			writebuffer[buffer_index++] = '\n';
			writebuffer[buffer_index++] = '\0';
			// we have a whole line, switch buffers around
			if (writebuffer == buffer1) {
				writebuffer = buffer2;
				readbuffer = buffer1;
			} else {
				writebuffer = buffer1;
				readbuffer = buffer2;
			}
			buffer_index = 0;
			new_message = 1;
		}
		// NOTE: is this why we had hangs? Because we used
		// USART_SendData() + block in both interrupt an normal
		// context?
		USART_SendData(USART1, ch);
		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
			/* nop */
		}
	}

//	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
//		/* Read one byte from the receive data register */ 
//		RxBuffer[RxCount++] = (USART_ReceiveData(USART1) & 0x7F);
//		if (RxCount == NbrOfDataToRead) {
//			/* Disable the USART1 Receive interrupt */ 
//			USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
//		}
//	}
//
//	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
//		/* Write one byte to the transmit data register */ 
//		USART_SendData(USART1, TxBuffer[TxCount++]);
//		if (TxCount == NbrOfDataToTransfer) {
//			/* Disable the USART1 Transmit interrupt */ 
//			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
//		}
//	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
