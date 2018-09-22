/*
 * Copyright 2011-2012 Ayla Networks, Inc.  All rights reserved.
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
#ifndef __AYLA_SPI_PLATFORM_ARCH_H__
#define __AYLA_SPI_PLATFORM_ARCH_H__

#if   defined(MCU_ST_STM32_CM4)
#define SPI_DEVICE	SPI2

#define INTR_N_GPIO	GPIOB
#define	INTR_N_PIN	11
#define INTR_N_EXT_LINE	EXTI_Line11
#define INTR_N_IRQ	EXTI15_10_IRQn
#define INTR_N_PIN_SOURCE GPIO_PinSource11

#define GP_SPI_GPIO	GPIOB
#define GP_SPI_NSS	12
#define GP_SPI_SCK	13
#define GP_SPI_MISO	14
#define GP_SPI_MOSI	15

/*
 * definitions needed from stm32f30x.h,
 * which can't be included because of typedef conflicts.
 */
#ifndef SPI_CR1_SPE
#define SPI_CR1_SPE	0x40
#define SPI_CR1_CRCNEXT	0x1000
#define SPI_CR1_CRCEN	0x2000

#define SPI_CR2_SSOE	0x04

#define SPI_SR_RXNE	0x01U
#define SPI_SR_TXE	0x02U
#define SPI_SR_CHSIDE	0x04U
#define SPI_SR_UDR	0x08U
#define SPI_SR_CRCERR	0x10U
#define	SPI_SR_MODF	0x20U
#define SPI_SR_OVR	0x40U
#define SPI_SR_BSY	0x80U
#endif

#ifndef SPI_I2SCFGR_I2SMOD
#define SPI_I2SCFGR_I2SMOD 0x800
#endif

#elif defined(MCU_ST_STM32_CM0)
#define SPI_DEVICE	SPI1

#define INTR_N_GPIO	GPIOB
//#define	INTR_N_PIN	6
#define	INTR_N_PIN	12	//huangjituan
//#define INTR_N_EXT_LINE	EXTI_Line6
#define INTR_N_EXT_LINE	EXTI_Line12	//huangjituan
#define INTR_N_IRQ	EXTI4_15_IRQn
//#define INTR_N_PIN_SOURCE GPIO_PinSource6
#define INTR_N_PIN_SOURCE GPIO_PinSource12		//huangjituan

/*
#define GP_SPI_GPIO_NSS	GPIOA
#define GP_SPI_NSS	15
#define GP_SPI_GPIO	GPIOB
#define GP_SPI_SCK	3
#define GP_SPI_MISO	4
#define GP_SPI_MOSI	5
*/
/* huangjituan */
#define GP_SPI_GPIO_NSS	GPIOA
#define GP_SPI_NSS	4
#define GP_SPI_GPIO	GPIOA
#define GP_SPI_SCK	5
#define GP_SPI_MISO	6
#define GP_SPI_MOSI	7


/*
 * definitions needed from stm32f30x.h,
 * which can't be included because of typedef conflicts.
 */
#ifndef SPI_CR1_SPE
#define SPI_CR1_SPE	0x40
#define SPI_CR1_CRCNEXT	0x1000
#define SPI_CR1_CRCEN	0x2000

#define SPI_CR2_SSOE	0x04

#define SPI_SR_RXNE	0x01U
#define SPI_SR_TXE	0x02U
#define SPI_SR_CHSIDE	0x04U
#define SPI_SR_UDR	0x08U
#define SPI_SR_CRCERR	0x10U
#define	SPI_SR_MODF	0x20U
#define SPI_SR_OVR	0x40U
#define SPI_SR_BSY	0x80U
#endif

#ifndef SPI_I2SCFGR_I2SMOD
#define SPI_I2SCFGR_I2SMOD 0x800
#endif

#elif defined(MCU_NXP_KINETIS_CM0P)
#define SPI_MODE_SELECTION  SPI_POLLING
/***************************************************************************//*!
 * @brief   SPI setting in 8bit POLLING mode
 * @details Baud rate is 1/24 of SYSCLK(SPI 1), 8-bit polling mode, module enabled 
 *          after initialization, master mode, SS is configured as GPIO 
 *          (must be driven manually!), no FIFO no DMA no MATCH functionality, 
 *          full-duplex mode.
 *          The initialization value of the configuration structure split by 
 *          peripheral registers is shown above.
 * @showinitializer
 ******************************************************************************/
#define SPI1_MODULE_DIV24_8B_CPHAHi_POLL_CONFIG                                \
(tSPI){                                                                        \
/* C1   */ CLR(SPI_C1_SPIE_MASK)|SET(SPI_C1_SPE_MASK)|CLR(SPI_C1_SPTIE_MASK)|  \
/* ..   */ SET(SPI_C1_MSTR_MASK)|CLR(SPI_C1_CPOL_MASK)|SET(SPI_C1_CPHA_MASK)|  \
/* ..   */ CLR(SPI_C1_SSOE_MASK)|CLR(SPI_C1_LSBFE_MASK),                       \
/* C2   */ CLR(SPI_C2_SPMIE_MASK)|CLR(SPI_C2_SPIMODE_MASK)|                    \
/* ..   */ CLR(SPI_C2_TXDMAE_MASK)|CLR(SPI_C2_MODFEN_MASK)|                    \
/* ..   */ CLR(SPI_C2_RXDMAE_MASK)|CLR(SPI_C2_SPISWAI_MASK)|                   \
/* ..   */ CLR(SPI_C2_SPC0_MASK),                                              \
/* C3   */ 0,   /* C3 is not used for SPI0 module due to no FIFO support */    \
/* BR   */ SET(SPI_BR_SPPR(2))|SET(SPI_BR_SPR(2)),                             \
/* ML   */ 0,                                                                  \
/* MH   */ 0,                                                                  \
}
/* Assgin PTC1, input, INTR_N signal input */
#define	INTR_N_PIN	PIN_1                 /* PTC1 */
/* SPI signal defines  */
#define SPI1_SS                         PIN_4            /* PTD4 */
#define SPI1_SCK                        PIN_5            /* PTD5 */
#define SPI1_MOSI                       PIN_6            /* PTD6 */
#define SPI1_MISO                       PIN_7            /* PTD7 */
#define GP_SPI_NSS	SPI1_SS
#define GP_SPI_SCK	SPI1_SCK
#define GP_SPI_MISO	SPI1_MISO
#define GP_SPI_MOSI	SPI1_MOSI
#endif

void spi_platform_init(void);
void spi_platform_intr_init(void);

#endif /*  __AYLA_SPI_PLATFORM_ARCH_H__ */
