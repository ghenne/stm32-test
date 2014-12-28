//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"

// ----------------------------------------------------------------------------
//
// STM32F0 led blink sample (trace via ITM).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 2 / 3)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"





#define BSRR_VAL 0x0400

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

void ir_pin_init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Configure PA15 in input mode, no pullup/down */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
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
	const int fuzzyness = 150; // we accept +-NN us pulse lengths
	int i;

	memset(parsed_data, 0, 4);

	// detect header
	if (!in_range(pulse_buffer[0], 1800, 200) && !in_range(pulse_buffer[1], 1800, 200)) {
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
		if (in_range(pulse_buffer[i+1], 650, fuzzyness)) {
			// a 'one'
			add_bit(parsed_data, (i-2)/2, 1);
		} else if (in_range(pulse_buffer[i+1], 300, fuzzyness)) {
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
		trace_printf("parsed data: %03d %03d %03d %03d\r\n",
				parsed_data[0],
				parsed_data[1],
				parsed_data[2],
				parsed_data[3]);
	}
}



int
main(int argc, char* argv[])
{
  // By customising __initialize_args() it is possible to pass arguments,
  // for example when running tests with semihosting you can pass various
  // options to the test.
  // trace_dump_args(argc, argv);

  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // The standard output and the standard error should be forwarded to
  // the trace device. For this to work, a redirection in _write.c is
  // required.
  puts("Standard output message.");

  // printing to stderr seems vastly slower...
  //fprintf(stderr, "Standard error message.\n");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %uHz\n", SystemCoreClock);

  timer_start();
  ir_pin_init();

  blink_led_init();
  
  uint32_t seconds = 0;

  // Infinite loop
  while (1)
    {
	  detect_and_print_pulses();

//      blink_led_on();
//      timer_sleep(BLINK_ON_TICKS);
//
//      blink_led_off();
//      timer_sleep(BLINK_OFF_TICKS);

      ++seconds;

      // Count seconds on the trace device.
      //trace_printf("Second %u\n", seconds);
    }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
