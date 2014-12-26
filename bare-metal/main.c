// By Wolfgang Wieser, heavily based on:
// http://fun-tech.se/stm32/OlimexBlinky/mini.php

#include <stdint.h>

static int global_data = 1;
static int global_bss;

//#include "stm32f0xx.h"
// Nah, the above file has dependencies, just copy in the needed definitions.

// BEGIN STM32f0xx.h stuff
/** 
  * @brief Reset and Clock Control
  */
typedef struct
{
  volatile uint32_t CR;         /*!< RCC clock control register,                                  Address offset: 0x00 */
  volatile uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  volatile uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  volatile uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  volatile uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  volatile uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  volatile uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  volatile uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
  volatile uint32_t BDCR;       /*!< RCC Backup domain control register,                          Address offset: 0x20 */ 
  volatile uint32_t CSR;        /*!< RCC clock control & status register,                         Address offset: 0x24 */
  volatile uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
  volatile uint32_t CFGR2;      /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
  volatile uint32_t CFGR3;      /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
  volatile uint32_t CR2;        /*!< RCC clock control register 2,                                Address offset: 0x34 */
} RCC_TypeDef;
#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000)
#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)
#define RCC_AHBENR_GPIOBEN    ((uint32_t)0x00040000)        /*!< GPIOB clock enable */
#define RCC_AHBENR_GPIOCEN    ((uint32_t)0x00080000)        /*!< GPIOC clock enable */


#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000)

#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)


/** 
  * @brief General Purpose IO
  */

typedef struct
{
  volatile uint32_t MODER;        /*!< GPIO port mode register,                                  Address offset: 0x00 */
  volatile uint16_t OTYPER;       /*!< GPIO port output type register,                           Address offset: 0x04 */
  uint16_t RESERVED0;         /*!< Reserved,                                                                 0x06 */
  volatile uint32_t OSPEEDR;      /*!< GPIO port output speed register,                          Address offset: 0x08 */
  volatile uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,                     Address offset: 0x0C */
  volatile uint16_t IDR;          /*!< GPIO port input data register,                            Address offset: 0x10 */
  uint16_t RESERVED1;         /*!< Reserved,                                                                 0x12 */
  volatile uint16_t ODR;          /*!< GPIO port output data register,                           Address offset: 0x14 */
  uint16_t RESERVED2;         /*!< Reserved,                                                                 0x16 */
  volatile uint32_t BSRR;         /*!< GPIO port bit set/reset registerBSRR,                     Address offset: 0x18 */
  volatile uint32_t LCKR;         /*!< GPIO port configuration lock register,                    Address offset: 0x1C */
  volatile uint32_t AFR[2];       /*!< GPIO alternate function low register,                Address offset: 0x20-0x24 */
  volatile uint16_t BRR;          /*!< GPIO bit reset register,                                  Address offset: 0x28 */
  uint16_t RESERVED3;         /*!< Reserved,                                                                 0x2A */
}GPIO_TypeDef;
#define GPIO_MODER_MODER8_0        ((uint32_t)0x00010000)
// END STM32f0xx.h stuff




#define LED_IR  7  // GPIOC, pin 7
#define LED_BLUE  8  // GPIOC, pin 8
#define LED_GREEN 9  // GPIOC, pin 9

#define STACK_TOP 0x20000800   // just a tiny stack for demo

static void nmi_handler(void);
static void hardfault_handler(void);
int main(void);

// Define the vector table
uint32_t *myvectors[4]
__attribute__ ((section("vectors"))) = {
	(uint32_t *) STACK_TOP,         // stack pointer
	(uint32_t *) main,              // code entry point
	(uint32_t *) nmi_handler,       // NMI handler (not really)
	(uint32_t *) hardfault_handler  // hard fault handler
};

void nmi_handler(void)
{
	for (;;);
}

void hardfault_handler(void)
{
	for (;;);
}

// the simplest program ever, need debugger to see it run
int main1(void)
{
	int i = 0;

	while (1) {
		i++;
	}
}


void init_gpio_led(void)
{
	// Enable GPIO port C, pin 8 and 9, as output.
	// See STM32F0xx reference manual.

	// enable clock to GPIO port C (must be done before writing to any GPIO registers!)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	//GPIOC->AFR = // defaults to GPIO?

	// set LED pins as output (CMSIS defines GPIO_MODER_MODER<pin>_0 for this)
	GPIOC->MODER  = 0x01 << (LED_IR*2);
	GPIOC->MODER |= 0x01 << (LED_BLUE*2);
	GPIOC->MODER |= 0x01 << (LED_GREEN*2);

	//GPIOC->OTYPER configures push-pull vs open drain. Default is push-pull.

	// Configure port output speed. Not important, anything will do when driving LEDs.
	//GPIOC->OSPEEDR =

	// pull-up/pull-down not relevant when driving LEDs.
	//GPIOC->PUPDR =
}

void led_blue(int enable)
{
	// set/clear LED
	// BSRR is a set/reset register. When writing a '1' in the upper
	// 16-bits of the register, the corresponding GPIO pin is cleared. When
	// writing to the lower 16-bits, the corresponding GPIO pin is set.
	// Each GPIO port has 16 pins.
	if (enable) {
		GPIOC->BSRR = 1 << LED_BLUE; // set LED
	} else {
		GPIOC->BSRR = (1 << LED_BLUE) << 16; // clear LED
		// there is also the GPIOC->BRR register, this is equivalent with the above
		//GPIOC->BRR = (1 << LED_BLUE); // clear LED
	}
}

void led_green(int enable)
{
	if (enable) {
		GPIOC->BSRR = 1 << LED_GREEN;
	} else {
		GPIOC->BSRR = (1 << LED_GREEN) << 16; // upper 16-bits clears corresponding pin
	}
}

void led_ir(int enable)
{
	if (enable) {
		GPIOC->BSRR = 1 << LED_IR;
	} else {
		GPIOC->BSRR = (1 << LED_IR) << 16; // upper 16-bits clears corresponding pin
	}
}

#define DELAY_US(microseconds) \
	do { \
		int us = microseconds; \
		while (us-- > 0) { \
			asm("nop"); \
			asm("nop"); \
			asm("nop"); \
			asm("nop"); \
			asm("nop"); \
			asm("nop"); \
			asm("nop"); \
			asm("nop"); \
			asm("nop"); \
			asm("nop"); \
		} \
	} while (0)

void delay_us(int us)
{
	while (us-- > 0) {
		asm("nop");
	}
}

void delay_ms(int ms)
{
	int i;

	while (ms-- > 0) {
		for (i = 0; i < 565; i++) {  // was 8000==1ms at work
			asm("nop"); // __NOP(); // defined in CMSIS, for many compilers
		}
	}
}

void send_header(void)
{
	int i;

	for (i = 0; i < 85; i++) {
		GPIOC->ODR |= (1 << LED_IR);
		delay_us(17);
		GPIOC->ODR &= ~(1 << LED_IR);
		delay_us(17);
	}
	delay_ms(2);
}

void send_one(void)
{
	int i;

	for (i = 0; i < 4; i++) {
		GPIOC->ODR |= (1 << LED_IR);
		delay_us(17);
		GPIOC->ODR &= ~(1 << LED_IR);
		delay_us(17);
	}
	delay_us(300);
}

void send_zero(void)
{
	int i;

	for (i = 0; i < 4; i++) {
		GPIOC->ODR |= (1 << LED_IR);
		delay_us(17);
		GPIOC->ODR &= ~(1 << LED_IR);
		delay_us(17);
	}
	delay_us(600);
}

void send_command(int throttle, int leftright, int forwardbackward)
{
	send_header();
	// TODO: finish...
	send_one();
	send_one();
	send_zero();
	send_zero();
	//send_zero();
	//send_one();
}


// Put clock output (MCO) on PA8
void set_clock_on_mco(void)
{
	const int no_clock = 0;
	const int hsi14 = 1;
	const int lsi = 2;
	const int lse = 3;
	const int sysclk = 4;
	const int hsi = 5;
	const int hse = 6;
	const int plldiv2 = 7;
	// Set MCO[2:0] bits
	RCC->CFGR &= ~(7 << 24);
	RCC->CFGR |= sysclk << 24;

	// Enable GPIOA clock so we can read/write its registers
	RCC->AHBENR |= 1 << 17;

	// Enable alternate function mode for PA8
	GPIOA->MODER |= 2 << (8*2);

	// set push-pull drive mode (default after reset)
	GPIOA->OTYPER &= ~(1 << (8));

	// no pull-up/pull-down
	GPIOA->PUPDR &= ~(3 << (8*2));

	// After reset all I/Os are connected to alternate function 0 (AF0).
	// This is the mode that MCO is at, so there is nothing more to do :-)
}

// use the PLL to boost sysclock from 8MHz to 48MHz
void clock_setup(void)
{
	// The MCU starts out with the HSI clock and no PLL. We set the PLL
	// multiplier, start the PLL and then switch to using PLLCLK instead of
	// HSI. PLLSRC defaults to HSI/2, so the PLL input is 4 MHz.

	RCC->CFGR &= ~(0xf << 18); // PLLMUL[3:0]: clear
	RCC->CFGR |= 10 << 18; // PLLMUL[3:0]: set PLL input clock multiplier

	// enable PLL
	RCC->CR |= 1 << 24; // PLLON = 1;
	// wait for PLLRDY flag
	while (!(RCC->CR & (1 << 25))) {
		/* empty */
	}

	RCC->CFGR |= 2 << 2; // SW (System clock switch): PLL selected as system clock

	// FIXME: the PLL is OK, but sysclock is still at 8MHz
}

int main()
{
	init_gpio_led();
	set_clock_on_mco();
	clock_setup();

	while (1) {
		//led_blue(1);
		//delay_us(100);
		//led_blue(0);
		//delay_ms(1);
		//delay_ms(1000);

		//led_green(1);
		//delay_ms(50);
		//led_green(0);
		//delay_ms(1000);

		GPIOC->ODR |= (1 << LED_BLUE);
		delay_us(1);
		GPIOC->ODR &= ~(1 << LED_BLUE);
		delay_us(1);

		//send_command(0, 0, 0);
		//delay_ms(180);
	}

	return 0;
}
