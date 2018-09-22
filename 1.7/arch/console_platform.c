/*
 * Copyright 2011 Ayla Networks, Inc.  All rights reserved.
 */
#include <stdlib.h>
#include <ayla/mcu_platform.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>

#include <ayla/console.h>

#define USART		USART1
#define USART_IRQ	USART1_IRQn
#define USART_IRQ_PRI	12
#define USART_GPIO	GPIOA
#define USART_RX_PIN	GPIO_Pin_9	/* pin pa9 */
#define USART_TX_PIN	GPIO_Pin_10	/* pin pa10 */
#define USART_SPEED	115200

#if 0
/*
 * Send next byte.
 * Caller has disabled interrupts and determined SR_TXE and tx_started.
 */
static void serial_tx_next(void)
{
	struct serial_state *state = &serial_state;
	int data;

	if (!state->tx_started || !state->get_tx) {
		USART->CR1 &= ~USART_CR1_TXEIE;
		return;
	}
	data = state->get_tx();
	if (data >= 0) {
		USART->DR = data;
		USART->CR1 |= USART_CR1_TXEIE;
	} else {
		serial_state.tx_started = 0;
		USART->CR1 &= ~USART_CR1_TXEIE;
	}
}

void USART1_irq(void)
{
	struct serial_state *state = &serial_state;
	u16 sr;
	u16 dr;

	sr = USART->SR;
	if (sr & USART_SR_RXNE) {
		dr = USART->DR;
		if (((sr & (USART_SR_NE | USART_SR_PE | USART_SR_FE)) == 0) &&
		    state->rx_intr) {
			state->rx_intr((u8)dr);
		}
	} else if (sr & USART_SR_ORE) {
		(void)USART->DR;
	}
	if (sr & USART_SR_TXE) {
		serial_tx_next();
	}
}

/*
 * Start transmit if there is anything to send.
 */
void serial_start_tx(void)
{
	if (!serial_state.inited) {
		return;
	}
	if (serial_state.tx_started) {
		return;
	}
	serial_state.tx_started = 1;
	if (USART->SR & USART_SR_TXE) {
		serial_tx_next();
	}
}
#endif

void console_platform_init(void)
{
	/* NVIC_InitTypeDef nvic_init; */
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t apbclock;
	uint32_t br;
	RCC_ClocksTypeDef RCC_ClocksStatus;

	/*
	 * Initialize I/O pins for USART1.
	 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);

	/* Initialize the TX */
	GPIO_InitStructure.GPIO_Pin = USART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(USART_GPIO, &GPIO_InitStructure);

	/* Initialize the RX */
	GPIO_InitStructure.GPIO_Pin = USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(USART_GPIO, &GPIO_InitStructure);

	/* Set alternate function 7 for USART pins */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
#if 0
	/* TODO: USE POLLING FOR NOW */
	/*
	 * Enable receive interrupts.
	 */
	USART->CR1 |= USART_CR1_RXNEIE;
	(void)USART->SR;
	(void)USART->DR;

	nvic_init.NVIC_IRQChannel = USART_IRQ;
	nvic_init.NVIC_IRQChannelPreemptionPriority = SERIAL_PRI;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
#endif

	/* Setup the USART */
	/* Use 1 Stop Bit (no change to CR2 register) */
	/* Enable rx and tx */
	USART->CR1 |= USART_CR1_TE | USART_CR1_RE;

	/* Configure the USART Baud Rate */
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	apbclock = RCC_ClocksStatus.PCLK2_Frequency;	
	br = apbclock / USART_SPEED;
	USART->BRR = ((br & 0xf) >> 1) | (br & ~0xf);

	/* Enable the UART */
	USART->CR1 |= USART_CR1_UE;
}

/*
 * Poll for data. If RXNE, receive the packet. If TXE, see if there's another
 * packet to transmit.
 */
void console_platform_poll(void)
{
	int data;

	if (USART->ISR & USART_ISR_RXNE) {
		console_recv(USART->RDR & 0xff);
	}
	if (USART->ISR & USART_ISR_TXE) {
		data = console_tx();
		if (data >= 0) {
			USART->TDR = (u8)data & 0xff;
		}
	}
}
