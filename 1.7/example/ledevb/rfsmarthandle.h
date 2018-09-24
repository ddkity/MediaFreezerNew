#ifndef __RFSMARTHANDLE_H__
#define __RFSMARTHANDLE_H__

/************����ͨ������궨��***************/
#define UartNOP  		(0)           		/*  ���ڽ��մ�������	*/
#define UartSOP  		(1)           		/*  ������ʼλ		*/
#define UartLEN		(2)			  /*  ���ճ���			*/
#define UartTYPE		(3)			  /*  �ҵ�����			*/
#define UartFCRC		(4)			  /*  ֡ͬ��У��	    */
#define UartRESV_H	(5)			  /*  ����λ1			*/
#define UartRESV_L	(6)			  /*  ����λ2			*/
#define UartMSGID	(7)			  /*  ��Ϣ��ʶ			*/
#define UartVER_H		(8)			  /*  ���Э��汾		*/
#define UartVER_L		(9)			  /*  �ҵ�Э��汾		*/
#define UartMSGTYPE	(10)		 	 /*  ��Ϣ���ͱ�ʶ		*/
#define UartMSG		(11)		  	/*  ��Ϣ��			*/
#define UartMSGCRC	(12)		 	 /*  ��ϢУ����		*/

#define M_HEAD  		(0xAA)	/* ֡ͷ */
#define M_TYPE		(0XCA)	/* �ҵ����� ���� */
#define M_RESVH		(0X00)	/* ����λ1 */
#define M_RESVL		(0X00)	/* ����λ2 */
#define M_VER_H 		(0x00)	/* ���Э��汾 */
#define M_VER_L 		(0x00)	/* �ҵ�Э��汾 */

#define MAX_RECV_DATALEN (80)		/* ���ڽ���������ݳ��ȶ��� */
#define MAX_SEND_DATALEN (80)		/* ���ڽ���������ݳ��ȶ��� */

void Uart1Init(unsigned int BaudRate);
void TIM3Init(unsigned short arr,unsigned short psc);
void MediaUartSendFormData(unsigned char MsgID, unsigned char CMD,unsigned char *SendFormData,unsigned char MsgLen);





#endif

