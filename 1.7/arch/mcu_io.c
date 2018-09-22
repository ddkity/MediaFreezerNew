/*
 * Copyright 2011-2013 Ayla Networks, Inc.  All rights reserved.
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
#if   defined(MCU_ST_STM32_CM4)
#include <stm32f3_discovery.h>
#elif defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif
#include "mcu_io.h"

void mcu_io_init(void)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	GPIO_InitTypeDef  GPIO_InitStructure;
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif
	int i;

#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	/* Enable GPIOA, GPIOB, and GPIOC clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif

	/* Configure LEDs */
#if   defined(MCU_ST_STM32_CM4)
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED7);
	STM_EVAL_LEDInit(LED8);
	STM_EVAL_LEDInit(LED9);
	STM_EVAL_LEDInit(LED10);
#elif defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#if 0	/* No LED hardware support. Dummy */
	PORT_Init(PORTE, PORT_MODULE_ALT1_MODE, LED_BLUE|LED_GREEN, 0, NULL);
	GPIO_Init(GPIOE, GPIO_PIN_OUTPUT, LED_BLUE|LED_GREEN);
	GPIO_Clr(GPIOE, LED_BLUE|LED_GREEN);
#endif
#endif

#if   defined(MCU_ST_STM32_CM4)
	/* Configure PA0 pin as input for BUTTON */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#elif defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif

#if   defined(MCU_ST_STM32_CM4)
	/* Configure PB10 pin as input for READY_N */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#elif defined(MCU_ST_STM32_CM0)
	/* Configure PB0 pin as input for READY_N */
	GPIO_InitStructure.GPIO_Pin = bit(READY_N_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(READY_N_GPIO, &GPIO_InitStructure);
#elif defined(MCU_NXP_KINETIS_CM0P)
	/* Configure PTC4 pin as input for READY_N */
	PORT_Init(PORTC, PORT_MODULE_ALT1_MODE, READY_N_PIN, 0, NULL);
	GPIO_Init(GPIOC, GPIO_PIN_INPUT, READY_N_PIN);
#endif

#ifndef AYLA_UART /* UART mode on module uses PA0/WKUP as UART_CTS input */
#if   defined(MCU_ST_STM32_CM4)
	/* Configure PB2 pin as output for WAKEUP */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#elif defined(MCU_ST_STM32_CM0)
	/* Configure PB1 pin as output for WAKEUP */
	GPIO_InitStructure.GPIO_Pin = bit(WKUP_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(WKUP_GPIO, &GPIO_InitStructure);
#elif defined(MCU_NXP_KINETIS_CM0P)
	/* Configure PTB0 pin as output for WAKEUP */
	PORT_Init(PORTB, PORT_MODULE_ALT1_MODE, WKUP_PIN, 0, NULL);
	GPIO_Init(GPIOB, GPIO_PIN_OUTPUT, WKUP_PIN);
	GPIO_Clr(GPIOB, WKUP_PIN);
#endif
#endif /* AYLA_UART */

#if   defined(MCU_ST_STM32_CM4)
	/* Configure PB0 pin as output for RESET_N */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#elif defined(MCU_ST_STM32_CM0)
	/* Configure PC13 pin as output for RESET_N */
	GPIO_InitStructure.GPIO_Pin = bit(RESET_N_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(RESET_N_GPIO, &GPIO_InitStructure);
#elif defined(MCU_NXP_KINETIS_CM0P)
	/* Configure PC7 pin as output for RESET_N */
	PORT_Init(PORTC, PORT_MODULE_ALT1_MODE, RESET_N_PIN, 0, NULL);
	GPIO_Init(GPIOC, GPIO_PIN_OUTPUT, RESET_N_PIN);
#endif

#if   defined(MCU_ST_STM32_CM4)
	/* Configure PC0 pin as input for BUTTON1 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#elif defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif

#if   defined(MCU_ST_STM32_CM4)
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
#elif defined(MCU_ST_STM32_CM0)
	GPIO_SetBits(RESET_N_GPIO, bit(RESET_N_PIN));
#elif defined(MCU_NXP_KINETIS_CM0P)
	GPIO_Set(GPIOC, RESET_N_PIN);
#endif
	for (i = 1000; i--; )
		;

#if   defined(MCU_ST_STM32_CM4)
	GPIO_SetBits(GPIOB, GPIO_Pin_2);
#elif defined(MCU_ST_STM32_CM0)
	GPIO_SetBits(WKUP_GPIO, bit(WKUP_PIN));
#elif defined(MCU_NXP_KINETIS_CM0P)
	GPIO_Set(GPIOB, WKUP_PIN);
#endif
}
