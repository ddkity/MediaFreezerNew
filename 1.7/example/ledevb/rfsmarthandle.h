#ifndef __RFSMARTHANDLE_H__
#define __RFSMARTHANDLE_H__

/************串口通信命令宏定义***************/
#define UartNOP  		(0)           		/*  串口接收错误或空闲	*/
#define UartSOP  		(1)           		/*  接收起始位		*/
#define UartLEN		(2)			  /*  接收长度			*/
#define UartTYPE		(3)			  /*  家电类型			*/
#define UartFCRC		(4)			  /*  帧同步校验	    */
#define UartRESV_H	(5)			  /*  保留位1			*/
#define UartRESV_L	(6)			  /*  保留位2			*/
#define UartMSGID	(7)			  /*  消息标识			*/
#define UartVER_H		(8)			  /*  框架协议版本		*/
#define UartVER_L		(9)			  /*  家电协议版本		*/
#define UartMSGTYPE	(10)		 	 /*  消息类型标识		*/
#define UartMSG		(11)		  	/*  消息体			*/
#define UartMSGCRC	(12)		 	 /*  消息校验码		*/

#define M_HEAD  		(0xAA)	/* 帧头 */
#define M_TYPE		(0XCA)	/* 家电类型 冰箱 */
#define M_RESVH		(0X00)	/* 保留位1 */
#define M_RESVL		(0X00)	/* 保留位2 */
#define M_VER_H 		(0x00)	/* 框架协议版本 */
#define M_VER_L 		(0x00)	/* 家电协议版本 */

#define MAX_RECV_DATALEN (80)		/* 串口接收最大数据长度定义 */
#define MAX_SEND_DATALEN (80)		/* 串口接收最大数据长度定义 */

void Uart1Init(unsigned int BaudRate);
void TIM3Init(unsigned short arr,unsigned short psc);
void MediaUartSendFormData(unsigned char MsgID, unsigned char CMD,unsigned char *SendFormData,unsigned char MsgLen);





#endif

