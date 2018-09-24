/*
 * Copyright 2014 Ayla Networks, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Ayla Networks, Inc.
 */

#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
#include <string.h>
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif
#include <ayla/mcu_platform.h>
#include <mcu_io.h>
#include <stm32.h>
#ifdef DEMO_UART
#include <ayla/uart.h>
#else
#include <spi_platform_arch.h>
#endif /* DEMO_UART */
#include <ayla/cmp.h>
#include <demo.h>

struct stm32_debounce {
	u8	val;		/* exported value */
	u8	raw;		/* last read value */
	u32	bounce_tick;	/* bouncing - ignore changes while this set */
	u32	off_tick;	/* on time extended while this is set */
};
#if   defined(MCU_ST_STM32_CM4)
static struct stm32_debounce stm32_button;
#elif defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif

struct intr_stats intr_stats;
volatile u32 tick;

/*
 * Delay
 */
void stm32_delay_time(u32 ms)
{
	u32 end_tick = tick + (ms * SYSTICK_HZ) / 1000;

	while (cmp_gt(end_tick, tick)) {
		;
	}
}

/*
 * Reset the module
 */
void stm32_reset_module(void)
{
	volatile int i;

	/*
	 * setup reset line to module, and pulse it.
	 */
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	RESET_N_GPIO->BRR = bit(RESET_N_PIN);
#elif defined(MCU_NXP_KINETIS_CM0P)
	GPIO_Clr(GPIOC, RESET_N_PIN);
#endif
	for (i = 10000; i--; )
		;
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	RESET_N_GPIO->BSRR = bit(RESET_N_PIN);
#elif defined(MCU_NXP_KINETIS_CM0P)
	GPIO_Set(GPIOC, RESET_N_PIN);
#endif
}

static void stm32_intr_init(void)
{
#if   defined(MCU_ST_STM32_CM4)
	NVIC_InitTypeDef nvic_init;
	EXTI_InitTypeDef exti_init;
	extern void *__Vectors;

	/*
	 * Make sure vector table offset is set correctly before enabling
	 * interrupts.
	 */
	SCB->VTOR = (u32)&__Vectors;

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	/* 4 bits priority */

#elif defined(MCU_ST_STM32_CM0)
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

#elif defined(MCU_NXP_KINETIS_CM0P)
#endif

#ifndef DEMO_UART
	spi_platform_intr_init();
#endif
#if   defined(MCU_ST_STM32_CM4)
	/*
	 * Set button to cause external interrupt on either edge.
	 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	exti_init.EXTI_Line = BUTTON_EXT_LINE;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

	nvic_init.NVIC_IRQChannel = BUTTON_IRQ;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 15;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
#elif defined(MCU_ST_STM32_CM0)
	/* No button hardware support. Dummy */
#elif defined(MCU_NXP_KINETIS_CM0P)
	/* No button hardware support. Dummy */
#endif
}

/*
 * Check if module is ready
 */
int stm32_ready(void)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	return (READY_N_GPIO->IDR & bit(READY_N_PIN)) == 0;
#elif defined(MCU_NXP_KINETIS_CM0P)
	return (GPIO_Get(GPIOC) & READY_N_PIN) == 0;
#endif
}

/*
 * Get the state of a LED
 */
int stm32_get_led(u32 pin)
{
#if   defined(MCU_ST_STM32_CM4)
	return ((pin & LED_GPIO->ODR) != 0);
#elif defined(MCU_ST_STM32_CM0)
	return 0;	/* No LED hardware support. Dummy */
#elif defined(MCU_NXP_KINETIS_CM0P)
	return 0;	/* No LED hardware support. Dummy */
#endif
}

/*
 * Set an LED to a value
 */
void stm32_set_led(u32 pin, u8 val)
{
#if   defined(MCU_ST_STM32_CM4)
	if (val) {
		LED_GPIO->BSRR = pin;
	} else {
		LED_GPIO->BRR = pin;
	}
#elif defined(MCU_ST_STM32_CM0)
	/* No LED hardware support. Dummy */
#elif defined(MCU_NXP_KINETIS_CM0P)
	/* No LED hardware support. Dummy */
#endif
}

/*
 * Set the LED used to indicate factory reset
 */
void stm32_set_factory_rst_led(int on)
{
#if   defined(MCU_ST_STM32_CM4)
	stm32_set_led(bit(LED0_PIN), on);
#elif defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif
}

static void stm32_systick_init(void)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	SysTick_Config(CPU_CLK_HZ / SYSTICK_HZ);
#elif defined(MCU_NXP_KINETIS_CM0P)
	SysTick_Config(CPU_CLK_HZ / (SYSTICK_HZ*2));
#endif
}

#if   defined(MCU_ST_STM32_CM4)
/*
 * Read pushbutton and debounce it.
 * Extend the on time to make it at least BUTTON_ON_MIN_TICKS long.
 * Don't report changes within BUTTON_DEBOUNCE_TICKS of a previous change.
 * Return the debounced, not the extended value.
 */
static u8 stm32_button_read(void)
{
	u8 new;

	new = (BUTTON_GPIO->IDR & bit(BUTTON_PIN)) != 0;
	if (stm32_button.raw == new) {
		return new;
	}
	stm32_button.raw = new;
	if (stm32_button.bounce_tick) {
		stm32_button.bounce_tick++;
		new = stm32_button.val;
	} else {
		stm32_button.bounce_tick = tick + BUTTON_DEBOUNCE_TICKS;
		if (!stm32_button.off_tick) {
			if (new) {
				stm32_button.off_tick =
				     tick + BUTTON_ON_MIN_TICKS;
			}
			stm32_button.val = new;
			#if 0
			demo_set_button_state(stm32_button.val);
			#endif
		}
	}
	return new;
}

/*
 * Button Interrupt Handler.
 */
void EXTI0_IRQHandler(void)
{
	intr_stats.button++;
	if (!EXTI_GetITStatus(BUTTON_EXT_LINE)) {
		return;
	}
	EXTI_ClearITPendingBit(BUTTON_EXT_LINE);
	stm32_button_read();
}
#elif defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
/* No button hardware support. Dummy */
#endif

void SysTick_Handler(void)
{
	tick++;
#if   defined(MCU_ST_STM32_CM4)
	if (stm32_button.bounce_tick && cmp_gt(tick, stm32_button.bounce_tick)) {
		stm32_button.bounce_tick = 0;
		stm32_button_read();
	}
	if (stm32_button.off_tick && cmp_gt(tick, stm32_button.off_tick)) {
		stm32_button.off_tick = 0;
		stm32_button_read();
		if (stm32_button.val > stm32_button.raw) {
			stm32_button.val = stm32_button.raw;
			#if 0
			demo_set_button_state(stm32_button.val);
			#endif
		}
	}
#elif defined(MCU_NXP_KINETIS_CM0P) || defined(MCU_ST_STM32_CM0)
#endif
}

/*
 * See if user is holding down the blue button during boot.
 * The button must be down for 5 seconds total.  After 1 second we
 * start blinking the blue LED.  After 5 seconds we do the reset.
 */
int stm32_factory_reset_detect(void)
{
#if   defined(MCU_ST_STM32_CM4)
	int next_blink = 0;
	int reset_time;
	int led = 0;

	reset_time = tick + SYSTICK_HZ * 5;
	while (cmp_gt(reset_time, tick)) {
		if (!stm32_button_read()) {
			stm32_set_factory_rst_led(0);
			return 0;
		}
		if (cmp_gt(tick, next_blink)) {
			next_blink = tick + SYSTICK_HZ / 8; /* 8 Hz blink */
			led ^= 1;
			stm32_set_factory_rst_led(led);
		}
	}
	stm32_set_factory_rst_led(1);
	return 1;
#elif defined(MCU_ST_STM32_CM0)
	return 0;
#elif defined(MCU_NXP_KINETIS_CM0P)
	return 0;
#endif
}

/*
 * Initialize the stm32
 */
void stm32_init(void)
{
	stm32_systick_init();
	stm32_intr_init();
}
