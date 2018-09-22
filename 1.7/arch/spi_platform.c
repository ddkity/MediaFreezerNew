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
#include "mcu_io.h"
#include "spi_platform_arch.h"
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#include "spi.h"
static void spi1_ss_callback (SS_CONTROL ss_pin)
{
	if (ss_pin == SS_SET)
	{
		GPIO_Set(GPIOD,GP_SPI_NSS );
	}
	else if (ss_pin == SS_CLEAR)
	{ 
		GPIO_Clr(GPIOD, GP_SPI_NSS);
	}
	else if (ss_pin == SS_INIT)
	{
		PORT_Init(PORTD, PORT_MODULE_ALT1_MODE, GP_SPI_NSS,0,NULL);
		GPIO_Init(GPIOD, GPIO_PIN_OUTPUT, GP_SPI_NSS);
		GPIO_Set(GPIOD, GP_SPI_NSS); 
	}
}
#if 0
static void port_callback(vuint32 pin_number)
{ 
	if (pin_number&PIN_1)
	{
		intr_stats.intr_line++;
	}
}
#endif
#endif

static void spi_platform_wait_idle(void)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	SPI_TypeDef *spi = SPI_DEVICE;

	/*
	 * Make sure SPI I/O is complete first.
	 * The reference manual says to check for TXE followed by not busy.
	 */
	while (!(spi->SR & SPI_I2S_FLAG_TXE))
		;
	while (spi->SR & SPI_I2S_FLAG_BSY)
		;
#elif defined(MCU_NXP_KINETIS_CM0P)
	while (!SPI_TxCompl(SPI1));	/* wait until Tx buffer is empty */
	while (SPI_RxFull(SPI1));	/* wait until Rx buffer is empty */
#endif
}

void spi_platform_init(void)
{
#if   defined(MCU_ST_STM32_CM4)
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable GPIOB clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15);

	/* Configure PB12,13,14,15 pins for SPI2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_5);

	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Configure SPI2 for Master mode */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_DEVICE, &SPI_InitStructure);

	SPI_RxFIFOThresholdConfig(SPI_DEVICE, SPI_RxFIFOThreshold_QF);

	/* Clear flags and any RX data */
	SPI_I2S_ReceiveData16(SPI_DEVICE);
	SPI_I2S_GetFlagStatus(SPI_DEVICE, 0x00);

	SPI_Cmd(SPI_DEVICE, ENABLE);

	/* Configure PB11 pin as input for INTR_N */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

#elif defined(MCU_ST_STM32_CM0)
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable GPIOA clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_SetBits(GP_SPI_GPIO_NSS, bit(GP_SPI_NSS));
	GPIO_SetBits(GP_SPI_GPIO, bit(GP_SPI_MISO) | bit(GP_SPI_MOSI));

	/* Configure PB4,5,6,7 pins for SPI1 */
	GPIO_InitStructure.GPIO_Pin = bit(GP_SPI_SCK) | bit(GP_SPI_MOSI);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* huangjituan */
	GPIO_Init(GP_SPI_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = bit(GP_SPI_NSS);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* huangjituan */
	GPIO_Init(GP_SPI_GPIO_NSS, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = bit(GP_SPI_MISO);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		/* huangjituan */
	GPIO_Init(GP_SPI_GPIO, &GPIO_InitStructure);

	GPIO_PinAFConfig(GP_SPI_GPIO_NSS,	GP_SPI_NSS,	GPIO_AF_0);
	GPIO_PinAFConfig(GP_SPI_GPIO,		GP_SPI_SCK,	GPIO_AF_0);
	GPIO_PinAFConfig(GP_SPI_GPIO,		GP_SPI_MISO,GPIO_AF_0);
	GPIO_PinAFConfig(GP_SPI_GPIO,		GP_SPI_MOSI,GPIO_AF_0);
	/* Enable the SPI clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Configure SPI1 for Master mode */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_DEVICE, &SPI_InitStructure);

	SPI_RxFIFOThresholdConfig(SPI_DEVICE, SPI_RxFIFOThreshold_QF);

	/* Clear flags and any RX data */
	SPI_I2S_ReceiveData16(SPI_DEVICE);
	SPI_I2S_GetFlagStatus(SPI_DEVICE, 0x00);

	SPI_Cmd(SPI_DEVICE, ENABLE);

	/* Configure PB12 pin as input for INTR_N */
	GPIO_InitStructure.GPIO_Pin = bit(INTR_N_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(INTR_N_GPIO, &GPIO_InitStructure);

#elif defined(MCU_NXP_KINETIS_CM0P)
	PORT_Init(PORTD, PORT_MODULE_ALT2_MODE, (GP_SPI_SCK|GP_SPI_MOSI|GP_SPI_MISO), 0, NULL);

	/* SCLK1 = 1/24 sysCLK = 2 MHz, MSTR = 1, CPOL = 0(LOW), CPHA = 1, SSOE = 0, LSBFE = 0(MSB first) */
	SPI1_Init (SPI1_MODULE_DIV24_8B_CPHAHi_POLL_CONFIG, spi1_ss_callback, 0, NULL);
#endif
}

void spi_platform_intr_init(void)
{
#if   defined(MCU_ST_STM32_CM4)
	NVIC_InitTypeDef nvic_init;
	EXTI_InitTypeDef exti_init;

	/*
	 * Set module interrupt line to cause ext interrupt on falling edge.
	 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);
	exti_init.EXTI_Line = INTR_N_EXT_LINE;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

	nvic_init.NVIC_IRQChannel = INTR_N_IRQ;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 15;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

#elif defined(MCU_ST_STM32_CM0)
	EXTI_InitTypeDef exti_init;

	/*
	 * Set module interrupt line to cause ext interrupt on falling edge.
	 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
	exti_init.EXTI_Line = INTR_N_EXT_LINE;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

#elif defined(MCU_NXP_KINETIS_CM0P)
	/* PTC1 configure as INTR_N input, trigger by falling edge, must configured to INT input*/
	PORT_Init(PORTC, PORT_MODULE_BUTTON_IRQFE_MODE, INTR_N_PIN, 1, NULL/* port_callback */);
	GPIO_Init(GPIOC, GPIO_PIN_INPUT, INTR_N_PIN);
#endif
}

/*
 * Select slave
 */
void spi_platform_slave_select(void)
{
#if   defined(MCU_ST_STM32_CM4)
	GP_SPI_GPIO->BRR = bit(GP_SPI_NSS);
#elif defined(MCU_ST_STM32_CM0)
	GP_SPI_GPIO_NSS->BRR = bit(GP_SPI_NSS);
#elif defined(MCU_NXP_KINETIS_CM0P)
	GPIO_Clr(GPIOD, GP_SPI_NSS);
#endif
}

/*
 * Deselect slave
 */
void spi_platform_slave_deselect(void)
{
	spi_platform_wait_idle();
#if   defined(MCU_ST_STM32_CM4)
	GP_SPI_GPIO->BSRR = bit(GP_SPI_NSS);
#elif defined(MCU_ST_STM32_CM0)
	GP_SPI_GPIO_NSS->BSRR = bit(GP_SPI_NSS);
#elif defined(MCU_NXP_KINETIS_CM0P)
	GPIO_Set(GPIOD, GP_SPI_NSS );
#endif
}

#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
static void spi_platform_out(u8 byte)
{
	SPI_TypeDef *spi = SPI_DEVICE;

	while (!(spi->SR & SPI_I2S_FLAG_TXE))
		;
	SPI_SendData8(SPI_DEVICE, byte);
}

static u8 spi_platform_in(void)
{
	SPI_TypeDef *spi = SPI_DEVICE;
	u16 sr;

	for (;;) {
		sr = spi->SR;
		if (sr & SPI_I2S_FLAG_OVR) {
			(void)spi->DR;
			(void)spi->SR;
		}
		if (sr & SPI_I2S_FLAG_RXNE) {
			return SPI_ReceiveData8(SPI_DEVICE);
		}
	}
}
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif

/*
 * Send out 'byte' and return the incoming byte
 */
u8 spi_platform_io(u8 byte)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	spi_platform_out(byte);
	return spi_platform_in();
#elif defined(MCU_NXP_KINETIS_CM0P)
	return SPI1_TxRxByte(byte);
#endif
}

#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
/*
 * Send and receive the last byte of a SPI message, followed by the CRC byte.
 *
 * NB:  The setting of CRCNEXT must happen immediately after the last
 * byte is sent.  See RM0008.
 */
extern u8 spi_crc_rx, spi_crc_tx;
u8 spi_platform_io_crc(u8 byte)
{
	SPI_TypeDef *spi = SPI_DEVICE;
	u16 sr;

	while (!(spi->SR & SPI_I2S_FLAG_TXE))
		;
	SPI_SendData8(SPI_DEVICE, byte);
	spi->CR1 |= SPI_CR1_CRCNEXT;
	byte = spi_platform_in();

	spi_crc_tx = spi->TXCRCR;

	/*
	 * Wait for CRC byte.
	 */
	for (;;) {
		sr = spi->SR;
		if (sr & SPI_I2S_FLAG_OVR) {
			SPI_ReceiveData8(SPI_DEVICE);
			(void)spi->SR;
		}
		if (sr & SPI_I2S_FLAG_RXNE) {
			spi_crc_rx = SPI_ReceiveData8(SPI_DEVICE);
			break;
		}
	}
	return byte;
}

/*
 * Enable CRC
 */
void spi_platform_crc_en(void)
{
	spi_platform_wait_idle();
	SPI_DEVICE->CR1 |= SPI_CR1_CRCEN;
}

/*
 * Clear CRC status and return if error
 */
int spi_platform_crc_err(void)
{
	SPI_TypeDef *spi = SPI_DEVICE;
	int err = 0;

	spi_platform_wait_idle();
	if (spi->SR & SPI_SR_CRCERR) {
		spi->SR &= ~SPI_SR_CRCERR;
		err = 1;
	}
	spi->CR1 &= ~(SPI_CR1_CRCEN | SPI_CR1_CRCNEXT);
	return err;
}
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif

#if   defined(MCU_ST_STM32_CM4)
/*
 * Interrupt handler for external interrupts 10 thru 15.
 * EXTI 11 is the module interrupt line, when asserted it means there is a
 * SPI message waiting to be received.
 */
void EXTI15_10_IRQHandler(void)
{
	intr_stats.intr_line++;
	if (!EXTI_GetITStatus(INTR_N_EXT_LINE)) {
		return;
	}
	EXTI_ClearITPendingBit(INTR_N_EXT_LINE);
}
#elif defined(MCU_ST_STM32_CM0)
/*
 * Interrupt handler for external interrupts 4 thru 15.
 * EXTI 12 is the module interrupt line, when asserted it means there is a
 * SPI message waiting to be received.
 */
void EXTI4_15_IRQHandler(void)
{
	intr_stats.intr_line++;
	if (!EXTI_GetITStatus(INTR_N_EXT_LINE)) {
		return;
	}
	EXTI_ClearITPendingBit(INTR_N_EXT_LINE);
}
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif

/*
 * Check if there is a receive pending
 */
int spi_platform_rx_pending(void)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	return (INTR_N_GPIO->IDR & bit(INTR_N_PIN)) == 0;
#elif defined(MCU_NXP_KINETIS_CM0P)
	return (GPIO_Get(GPIOC) & INTR_N_PIN) == 0;
#endif
}
