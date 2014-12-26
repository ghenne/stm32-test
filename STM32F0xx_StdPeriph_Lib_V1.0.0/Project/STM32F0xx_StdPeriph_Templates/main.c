/**
 ******************************************************************************
 * @file    GPIO/IOToggle/main.c 
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    18-May-2012
 * @brief   Main program body
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stm32f0xx.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
 * @{
 */

/** @addtogroup IOToggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BSRR_VAL 0x0400

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef        GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
 * Init USART1: USART1_TX on PA9, USART1_RX on PA10
 *
 * GPIO registers control pin functionality. To enable USART, we must configure
 * both the USART peripheral (on the APB bus) and the corresponding GPIO (on
 * the AHB bus). But first, we must enable the peripheral clocks.
 *
 * NOTE: inspired by STM_EVAL_COMInit()
 *
 * Consult the STM32F05xxx Reference Manual and STM32F051xx Datasheet for details.
 */
void usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
}

void usart1_send_char(char ch)
{
	USART_SendData(USART1, ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
		/* nop */
	}
}

// return 1 if something was received, else 0
int usart1_receive_char(char *ch)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
		*ch = USART_ReceiveData(USART1);
		return 1;
	} else {
		return 0;
	}
}

void usart1_send_str(char *str)
{
	while (*str != '\0') {
		usart1_send_char(*(str++));
	}
}

void delay_us(int us)
{
	while (us-- > 0) {
		// __NOP() is defined in CMSIS, for many compilers
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
}

void delay_ms(int ms)
{
	int i;

	while (ms-- > 0) {
		for (i = 0; i < 8000; i++) {
			__NOP(); // defined in CMSIS, for many compilers
		}
	}
}

void send_one(void)
{
	int i;

	for (i = 0; i < 12; i++) {
		GPIOC->BSRR = BSRR_VAL;
		delay_us(13);
		GPIOC->BRR = BSRR_VAL;
		delay_us(13);
	}
	delay_us(700);
}

void send_zero(void)
{
	int i;

	for (i = 0; i < 11; i++) {
		GPIOC->BSRR = BSRR_VAL;
		delay_us(13);
		GPIOC->BRR = BSRR_VAL;
		delay_us(13);
	}
	delay_us(330);
}

void send_header(void)
{
	int i;

	for (i = 0; i < 78; i++) {
		GPIOC->BSRR = BSRR_VAL;
		delay_us(13);
		GPIOC->BRR = BSRR_VAL;
		delay_us(13);
	}
	delay_ms(2);

	/* there is also this extra bit... */
	//send_zero();
}

void send_command(int leftright, int forwardbackward, int throttle, int trim)
{
	int i;

	send_header();

	for (i = 7; i >= 0; i--) {
		if (leftright & (1 << i)) {
			send_one();
		} else {
			send_zero();
		}
	}

	for (i = 7; i >= 0; i--) {
		if (forwardbackward & (1 << i)) {
			send_one();
		} else {
			send_zero();
		}
	}

	for (i = 7; i >= 0; i--) {
		if (throttle & (1 << i)) {
			send_one();
		} else {
			send_zero();
		}
	}

	for (i = 7; i >= 0; i--) {
		if (trim & (1 << i)) {
			send_one();
		} else {
			send_zero();
		}
	}

	/* there is actually a 1 bit footer */
	send_one();
}

int __io_putchar(char ch)
{
	usart1_send_char(ch);
	return 0;
}


int ir_pin_is_high(void)
{
	return (GPIOA->IDR & (1 << 15)) != (1 << 15); // PA15, inverted
	//return GPIOA->IDR & (1 << 15); // PA15, non-inverted
}

/**
 * Sample a IR pulse train
 *
 * Sample 0,2,4... is the duration of the high pulse (in microseconds)
 * Sample 1,3,5... is the duration of the low pulse (in microseconds)
 * 
 * @param buffer Where to store the samples
 * @param max_samples Max number of samples to store in buffer (or else, buffer overflow)
 * @param num_samples pointer to a variable that will contain the number of
 * samples captured (sampling stops when a low (or high) pulse of more than 10 ms is
 * detected)
 */
void detect_pulses(int *buffer, int max_samples, int *num_samples)
{
	int i;
	int sample;

	*num_samples = 0;

	/* check that pin is low for at least 10 ms */
	for (i = 0; i < 1000; i++) {
		if (ir_pin_is_high()) {
			return;
		}
		delay_us(10);
		i++;
	}

	/* block until pin is high (timeout after 200 ms) */
	i = 0;
	while (!ir_pin_is_high()) {
		// timeout after 200 ms (20000 * 10 us)
		if (i >= 20000) {
			return;
		}
		delay_us(10);
		i++;
	}

	for (sample = 0; sample < max_samples - 1; sample += 2) {
		/*
		 * Detect high pulse
		 */
		buffer[sample] = -1;

		/* block until pin goes high (timeout after 10 ms) */
		i = 0;
		while (!ir_pin_is_high()) {
			// timeout after 10 ms (1000 * 10 us)
			if (i >= 1000) {
				return;
			}
			delay_us(10);
			i++;
		}

		// for how long is this pulse high? timeout after N ms
		i = 0;
		while (ir_pin_is_high()) {
			// timeout after 10 ms (1000 * 10 us)
			if (i >= 1000) {
				buffer[sample] = -1;
				*num_samples = sample;
				return;
			}
			delay_us(10);
			i++;
		}
		// store the duration in microseconds
		buffer[sample] = i*10;

		/*
		 * Detect low pulse
		 */
		buffer[sample+1] = -1;

		// for how long is this pulse low? timeout after N ms
		i = 0;
		while (!ir_pin_is_high()) {
			// timeout after 10 ms (1000 * 10 us)
			if (i >= 1000) {
				buffer[sample+1] = -1;
				*num_samples = sample;
				return;
			}
			delay_us(10);
			i++;
		}
		// store the duration in microseconds
		buffer[sample+1] = i*10;
	}
}

// return 1 if in range, else 0
int in_range(int sample, int target, int fuzzyness)
{
	if (sample > target + fuzzyness || sample < target - fuzzyness) {
		return 0;
	}
	return 1;
}

void add_bit(uint8_t *accumulated, int bit_nr, char this_bit)
{
	if (this_bit) {
		accumulated[bit_nr/8] |= 1 << (8 - (bit_nr % 8));
	} else {
		accumulated[bit_nr/8] &= ~(1 << (8 - (bit_nr % 8)));
	}
}

// parsed_data is a 4 byte buffer.
// Return 0 if success, -1 on error
int parse_pulses(int *pulse_buffer, int num_samples, uint8_t *parsed_data)
{
	const int fuzzyness = 120; // we accept +-NN us pulse lengths
	int i;

	memset(parsed_data, 0, 4);

	// detect header
	if (!in_range(pulse_buffer[0], 2000, 200) && !in_range(pulse_buffer[1], 2000, 200)) {
		printf("broke out at header detection\r\n");
		goto err;
	}

	for (i = 2; i < num_samples; i += 2) {
		// detect high part
		if (in_range(pulse_buffer[i], 350, fuzzyness)) {
			// good
		} else {
			printf("broke out at i=%d (high part) %d\r\n", i, pulse_buffer[i]);
			goto err;
		}

		// detect low part
		if (in_range(pulse_buffer[i+1], 600, fuzzyness)) {
			// a 'one'
			add_bit(parsed_data, (i-2)/2, 1);
		} else if (in_range(pulse_buffer[i+1], 200, fuzzyness)) {
			// a 'zero'
			add_bit(parsed_data, (i-2)/2, 0);
		} else {
			printf("broke out at i+1=%d (low part) %d\r\n", i+1, pulse_buffer[i+1]);
			goto err;
		}
	}

	return 0;
err:
	for (i = 0; i < num_samples; i++) {
		printf("pulse_buffer[%d]: %d\r\n", i, pulse_buffer[i]);
	}
	return -1;
}

void detect_and_print_pulses(void)
{
	uint8_t parsed_data[4];
	int pulse_buffer[100];
	int num_samples;
	int ret;
	int i;

	detect_pulses(pulse_buffer, 100, &num_samples);
	//printf("detected a pulse train of length %d\r\n", num_samples);
	//for (i = 0; i < num_samples; i++) {
		//printf("pulse_buffer[%d]: %d\r\n", i, pulse_buffer[i]);
	//}
	if (num_samples != 66) {
		return;
	}
	ret = parse_pulses(pulse_buffer, num_samples, parsed_data);
	if (ret == 0) {
		printf("parsed data: %03d %03d %03d %03d\r\n",
				parsed_data[0],
				parsed_data[1],
				parsed_data[2],
				parsed_data[3]);
	}
}

void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

extern uint8_t buffer1[100];
extern uint8_t buffer2[100];
extern volatile uint8_t *readbuffer; // pointer to current read buffer
extern volatile uint8_t *writebuffer; // pointer to current write buffer
extern volatile uint8_t new_message; // set to one when we have a new line, the reader/consumer must clear it


// for baselibc
static size_t usart1_write(FILE *instance, const char *bp, size_t n)
{
	int i;
	for (i = 0; i < n; i++) {
		USART_SendData(USART1, bp[i]);
		// wait until it has been transmitted
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
			/* nop */
		}
	}
	return n;
}

static struct File_methods stdio_methods = {
        &usart1_write, NULL
};

static struct File _stdout = {
        &stdio_methods
};

static struct File _stderr = {
        &stdio_methods
};

FILE* const stdout = &_stdout;
FILE* const stderr = &_stderr;


typedef union usart_isr_u {
	struct {
		uint8_t pe        :1;
		uint8_t fe        :1;
		uint8_t nf        :1;
		uint8_t ore       :1;
		uint8_t idle      :1;
		uint8_t rxne      :1;
		uint8_t tc        :1;
		uint8_t txe       :1;
		uint8_t lbdf      :1;
		uint8_t ctsif     :1;
		uint8_t cts       :1;
		uint8_t rtof      :1;
		uint8_t eobf      :1;
		uint8_t reserved0 :1;
		uint8_t abre      :1;
		uint8_t abrf      :1;
		uint8_t busy      :1;
		uint8_t cmf       :1;
		uint8_t sbkf      :1;
		uint8_t rwu       :1;
		uint8_t wuf       :1;
		uint8_t teack     :1;
		uint8_t reack     :1;
	};
	uint32_t raw;
} usart_isr_t;

volatile usart_isr_t *usart1_isr = &(USART1->ISR);

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	int i;
	char ch = 'a';
	int leftright, forwardbackward, throttle, trim;
	char buffer[100];
	int matches;

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* Configure PC10 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PA15 in input mode, no pullup/down */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	usart1_init();
        // USART_IT_RXNE:  Receive Data register not empty interrupt.
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	//NVIC_Config();

	// disable error interrupts from USART1 (EIE=0)
	//USART1->CR3 &= ~(1 << 0);

	leftright = 126;
	forwardbackward = 126;
	throttle = 0;
	trim = 148;

	if (!writebuffer) {
		writebuffer = buffer1;
		readbuffer = buffer2;
	}

	{
		/* uint32_t dest[10]; */
		/* uint32_t src[10] = {0,1,2,3,4,5,6,7,8,9}; */
		/* __aeabi_memcpy(dest, src, 10); */
		/* memcpy(dest, src, 10); */
	}

	printf("Starting...\r\n"); 
	i = 20;
	while (1) {
		//send_command(132, 126, 24, 150);
		//send_command(leftright, forwardbackward, throttle, trim);
		// Channel command intervals:
		// A: 120 ms
		// B: 180 ms
		// But it seems that the heli doesn't care, so we can have longer intervals
		//delay_ms(180);

		if (usart1_receive_char(&ch)) {
			usart1_send_char(ch);
		}
		//delay_ms(1000);

		// USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		// if (new_message) {
			// strcpy(buffer, readbuffer);
			// new_message = 0;
			// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			// printf("new message: \"%s\"\r\n", buffer);
			// matches = sscanf((const char *)buffer, "%d %d %d", &leftright, &forwardbackward, &throttle);
			// if (matches == 3) {
				// printf("new controls: %d %d %d\r\n", leftright, forwardbackward, throttle);
			// }
		// } else {
			// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		// }

		//detect_and_print_pulses();
		//delay_ms(500);

//		GPIOC->BSRR = BSRR_VAL;
//		delay_us(600);
//		GPIOC->BRR = BSRR_VAL;
//		delay_us(10);
	}
}
