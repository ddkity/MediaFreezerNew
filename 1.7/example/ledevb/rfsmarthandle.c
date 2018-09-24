#include <string.h>
#include "stm32f0xx.h"
#include "rfsmarthandle.h"

unsigned char  UratRecDataTmp = 0;		/* 串口中断接收字符 */
unsigned char UartStatus = UartNOP;
unsigned char UartRxOkFlag = 0; 			/* 串口接受完一帧数据的标志，使用完成数据后清零 */
unsigned char UartRecvMsgLen = 0;		/* 接收到的消息体数据帧的长度 */
unsigned char UartRecvDataLen = 0;	 /* 接收到数据数长度计数器 */
unsigned char UartRecvData[MAX_RECV_DATALEN];		/* 串口接收的数据 */
unsigned char CalCRC_Frame = 0x00;		/* 计算帧同步校验结果 */
unsigned char CalCRC_Msg = 0x00;		/* 计算消息校验码结果 */
unsigned char TimerUartCnt = 0x00;		/* 串口接收计时，防止接收失败进入死循环 */
/*******************************************************************************
*	UART Init and data recv or send driver
*******************************************************************************/

void UartValueInit(void)
{
	UratRecDataTmp = 0x00;
	UartStatus = UartNOP;
	UartRxOkFlag = 0;
	UartRecvDataLen = 0;
	memset(UartRecvData, 0x00, MAX_RECV_DATALEN);
	CalCRC_Frame = 0x00;
	CalCRC_Msg = 0x00;
	UartRecvMsgLen = 0;
	TimerUartCnt = 0x00;
}
void USART1SendData(unsigned char *Data,unsigned char Lenth)
{
	unsigned char i;
	unsigned int cnt;

	for(i=0; i < Lenth; i++)
	{
		USART_SendData(USART1, Data[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		{
			cnt++;
		}
	}
} 

void USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		UratRecDataTmp =USART_ReceiveData(USART1);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		
		switch(UartStatus)
		{
			case UartNOP:
				if(UartRxOkFlag){
					break;
				}else{
					UartStatus = UartSOP;
				}

			case UartSOP:
				if(UratRecDataTmp == M_HEAD)
				{
					UartRecvDataLen = 0;
					UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
					TimerUartCnt = 0x01;
					UartStatus = UartLEN;
				}
				break;

			case UartLEN:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				CalCRC_Msg += UratRecDataTmp;
				CalCRC_Frame = UratRecDataTmp;
				UartRecvMsgLen = UratRecDataTmp - 10;		/* 计算消息体的长度 */
				UartStatus = UartTYPE;
				break;

			case UartTYPE:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				if(UratRecDataTmp == M_TYPE){
					CalCRC_Msg += UratRecDataTmp;
					CalCRC_Frame ^= UratRecDataTmp;
					UartStatus = UartFCRC;
				}else{
					UartValueInit();
				}
				
				break;

			case UartFCRC:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				CalCRC_Msg += UratRecDataTmp;
				if(CalCRC_Frame == UratRecDataTmp){
					UartStatus = UartRESV_H;
				}else{
					UartValueInit();
				}
				break;

			case UartRESV_H:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				if(UratRecDataTmp == M_RESVH){
					CalCRC_Msg += UratRecDataTmp;
					UartStatus = UartRESV_L;
				}else{
					UartValueInit();
				}
				break;

			case UartRESV_L:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				if(UratRecDataTmp == M_RESVL){
					CalCRC_Msg += UratRecDataTmp;
					UartStatus = UartMSGID;
				}else{
					UartValueInit();
				}
				break;

			case UartMSGID:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				CalCRC_Msg += UratRecDataTmp;
				UartStatus = UartVER_H;
				break;

			case UartVER_H:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				if(UratRecDataTmp == M_VER_H){
					CalCRC_Msg += UratRecDataTmp;
					UartStatus = UartVER_L;
				}else{
					UartValueInit();
				}
				break;

			case UartVER_L:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				if(UratRecDataTmp == M_VER_L){
					CalCRC_Msg += UratRecDataTmp;
					UartStatus = UartMSGTYPE;
				}else{
					UartValueInit();
				}
				break;

			case UartMSGTYPE:
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				CalCRC_Msg += UratRecDataTmp;
				UartStatus = UartMSG;
				break;

			case UartMSG:
				if(UartRecvMsgLen > 0){
					UartRecvMsgLen--;
					UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
					CalCRC_Msg += UratRecDataTmp;
					break;
				}else{
					UartStatus = UartMSGCRC;
				}

			case UartMSGCRC:
				UartRxOkFlag = 0x01;
				UartRecvData[UartRecvDataLen++] = UratRecDataTmp;
				CalCRC_Msg = (CalCRC_Msg^0xFF) + 1;
				if(CalCRC_Msg == UratRecDataTmp){
					//USART1SendData(UartRecvData, UartRecvDataLen);	/* debug，收到的数据原路返回 */
					/* 处理接收到的串口数据 */
				}
				UartValueInit();
				break;

			default:
				UartValueInit();
				break;
		}
	}
}

/* 长度只记消息体的长度 */
void MediaUartSendFormData(unsigned char MsgID, unsigned char CMD,unsigned char *SendFormData,unsigned char MsgLen)
{
	unsigned char SendData[MAX_SEND_DATALEN] = {0};
	unsigned char DataLen = 0;
	
	volatile unsigned char CRCTemp = 0;
	unsigned char i;
	
	SendData[DataLen++] = M_HEAD;
	
	SendData[DataLen++] = MsgLen + 10;
	CRCTemp += SendData[DataLen - 1];
	
	SendData[DataLen++] = M_TYPE;
	CRCTemp += SendData[DataLen - 1]; 
	
	SendData[DataLen++] = (MsgLen + 10) ^ M_TYPE;		/* 计算CRC */
	CRCTemp += SendData[DataLen - 1];
	
	SendData[DataLen++] = M_RESVH;
	CRCTemp += SendData[DataLen - 1];
	
	SendData[DataLen++] = M_RESVL;
	CRCTemp += SendData[DataLen - 1];
	
	SendData[DataLen++] = MsgID;		/* 消息标识 */
	CRCTemp += SendData[DataLen - 1];
	
	SendData[DataLen++] = M_VER_H;
	CRCTemp += SendData[DataLen - 1];
	
	SendData[DataLen++] = M_VER_L;
	CRCTemp += SendData[DataLen - 1];
	
	SendData[DataLen++] = CMD;
	CRCTemp += SendData[DataLen - 1];

	for(i=0; i<MsgLen; i++)
	{
		SendData[DataLen++] = SendFormData[i];
		CRCTemp += SendData[DataLen - 1];
	}

	SendData[DataLen++] = (CRCTemp ^ 0xFF) + 1;	/* 计算CRC */
	
	USART1SendData(SendData,DataLen);
}


void Uart1Init(unsigned int BaudRate)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
    
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);

	/* RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		/* handler */
		if(TimerUartCnt > 0){
			TimerUartCnt++;
		}else if(TimerUartCnt > 2){	/* 300ms还没有接收完成，串口恢复默认设置 */
			UartValueInit();
		}
	}
}

void TIM3Init(unsigned short arr,unsigned short psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM3, ENABLE);
}

/*******************************************************************************
*	UART data handler
*******************************************************************************/








