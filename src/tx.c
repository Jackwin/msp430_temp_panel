/*===========================================================================
* ��ַ ��http://www.cdebyte.com/   http://yhmcu.taobao.com/                 *
* ���� ������  ԭ �ں͵��ӹ�����  �� �ڰ��ص��ӿƼ����޹�˾                 * 
* �ʼ� ��yihe_liyong@126.com                                                *
* �绰 ��18615799380                                                        *
============================================================================*/

#include "tx.h"
#include "bsp.h"
#include "TempSensorMode.h"

// ��������
#define TX              1       // ����ģʽ
#define RX              0       // ����ģʽ

#define SEND_GAP        1000    // ÿ���1s����һ������
#define RECV_TIMEOUT    800     // ���ճ�ʱ

#define ACK_LENGTH      10      // Ӧ���źų���        
#define SEND_LENGTH     10      // ��������ÿ���ĳ���

INT8U   Cnt1ms = 0;             // 1ms����������ÿ1ms��һ 
INT8U   SendFlag = 0;           // =1�������������ݣ�=0������

INT16U  SendTime = 1;           // �������ݷ��ͼ��ʱ��
INT16U  RecvWaitTime = 0;       // ���յȴ�ʱ��                
INT16U  SendCnt = 0;            // �������͵����ݰ���                

// ��Ҫ���͵�����  
INT8U   SendBuffer[SEND_LENGTH] = {25, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

// ������ر���
INT8U   COM_TxNeed = 0;
INT8U   COM_RxCounter = 0;
INT8U   COM_TxCounter = 0;
INT8U   COM_RxBuffer[65] = { 0 };
INT8U   COM_TxBuffer[65] = { 0 };

//#define USRAT_SendByte()    USART_SendData8(COM_TxBuffer[COM_TxCounter++])
//#define USRAT_RecvByte()    COM_RxBuffer[COM_RxCounter++]=USART_ReceiveData8()

/*===========================================================================
* ���� : DelayMs() => ��ʱ����(ms��)                                        *
* ���� ��x, ��Ҫ��ʱ����(0-255)                                             *
============================================================================*/
//void DelayMs(INT8U x)
//{
//    Cnt1ms = 0;
//    while (Cnt1ms <= x);
//}

/*===========================================================================
* ���� ��TIM3_1MS_ISR() => ��ʱ��3������, ��ʱʱ���׼Ϊ1ms               *
============================================================================*/
//void TIM3_1MS_ISR(void)
//{
//    Cnt1ms++;
//
//    if (0 != RecvWaitTime)      { RecvWaitTime--; }     // ���ݽ��ռ�ʱ
//
//    if (0 != SendTime)      // 1msʱ�䵽����λSendFlag��־����������ѯ��������
//    {
//        if (--SendTime == 0)    { SendTime = SEND_GAP; SendFlag = 1; }
//    }
//}

/*=============================================================================
* ���� : USART_Send() => ͨ�����ڷ�������                                     *
* ���� ��buff, ����������     size�� ���ͳ���                                *
=============================================================================*/
//void USART_Send(INT8U *buff, INT8U size)
//{
//    if (size == 0)          { return; }
//
//    COM_TxNeed = 0;
//
//    while (size --)         { COM_TxBuffer[COM_TxNeed++] = *buff++; }
//
//    COM_TxCounter = 0;
//    USART_ITConfig(USART_IT_TXE, ENABLE);
//}

/*=============================================================================
* ����:  USART_RX_Interrupt() => ���ڽ����ж�                                 *
=============================================================================*/
//void USART_RX_Interrupt(void)
//{
//    USRAT_RecvByte();
//}

/*=============================================================================
* ����:  USART_TX_Interrupt() =>���ڷ����ж�
=============================================================================*/
//void USART_TX_Interrupt(void)
//{
//    if (COM_TxCounter < COM_TxNeed)     { USRAT_SendByte(); }
//    else
//    {
//        USART_ITConfig(USART_IT_TC, ENABLE);
//        USART_ITConfig(USART_IT_TXE, DISABLE);
//
//        if (USART_GetFlagStatus(USART_FLAG_TC))
//        {
//            USART_ITConfig(USART_IT_TC, DISABLE);
//            COM_TxNeed = 0;
//            COM_TxCounter = 0;
//        }
//    }
//}

/*===========================================================================
* ���� ��MCU_Initial() => ��ʼ��CPU����Ӳ��                                 *
* ˵�� ����������Ӳ���ĳ�ʼ���������Ѿ�������C�⣬��bsp.c�ļ�               *
============================================================================*/
void MCU_Initial(void)
{
    //SClK_Initial();         // ��ʼ��ϵͳʱ�ӣ�16M
    GPIO_Initial();         // ��ʼ��GPIO                  
    //TIM3_Initial();         // ��ʼ����ʱ��3����׼1ms
    //USART_Initial();        // ��ʼ������
    //SPI_Initial();          // ��ʼ��SPI

    //enableInterrupts();     // �����ж�
}

/*===========================================================================
* ���� ��RF_Initial() => ��ʼ��RFоƬ                                       *
* ���� ��mode, =0,����ģʽ�� else,����ģʽ                                  *
* ˵�� ��SI44XX�Ĳ������Ѿ�������C�⣬��SI446X.c�ļ��� �ṩSPI��CSN������	*
         ���ɵ������ڲ����к����û������ٹ���SI44XX�ļĴ����������⡣       *
============================================================================*/
void RF_Initial(INT8U mode)
{
	SI446X_RESET();         // SI446X ģ�鸴λ
    SI446X_CONFIG_INIT();   // �Ĵ�����ʼ�����Ĵ�������WDS���õ�ͷ�ļ�
    SI446X_SET_POWER(0x4F); // �������������Ϊ���
    SI446X_START_RX(0, 0, PACKET_LENGTH, 8, 8, 8);  // �������ģʽ         
}

/*===========================================================================
* ����: System_Initial() => ��ʼ��ϵͳ��������                              *
============================================================================*/
void System_Initial(void)
{
    MCU_Initial();      // ��ʼ��CPU����Ӳ��
    RF_Initial(TX);     // ��ʼ������оƬ,����ģʽ           
}

/*===========================================================================
* ���� : BSP_RF_SendPacket() => ���߷������ݺ���                            *
* ���� ��Sendbufferָ������͵����ݣ�length�������ݳ���                     *
* ��� ��0������ʧ�ܣ�else�����ͳɹ�                                        *
============================================================================*/
INT8U RF_SendPacket(INT8U *Sendbuffer, INT8U length)
{
//    INT8U ack_flag = 0;         // =1,���յ�Ӧ���źţ�=0������
//    INT8U error = 0, i=0, ack_len=0, ack_buffer[65]={ 0 };
    INT8U TxBuffer[100];
    
    SI446X_SEND_PACKET(Sendbuffer, length, 0, 0);   // ��������    
    do
    {
        SI446X_INT_STATUS(TxBuffer);
    }while (!(TxBuffer[3] & (1<<5)));               // �ȴ��������
    
    //LED0_TOG();
    //USART_Send("Transmit ok\r\n", 13);
    
//    SI446X_START_RX(0, 0, PACKET_LENGTH, 8, 8, 8);  // �������ģʽ���ȴ�Ӧ��
//    RecvWaitTime = RECV_TIMEOUT;                    // �ȴ�Ӧ��ʱ����Ϊ800ms
//
//    while (0 != RecvWaitTime)
//    {
//        SI446X_INT_STATUS(ack_buffer);              // ����Ƿ��յ����ݰ�
//
//        if (ack_buffer[3] & (1<<4))                 // ���Ϊ1��˵���յ����ݰ�
//        {
//            ack_len = SI446X_READ_PACKET(ack_buffer);   // ��ȡ�յ�������
//
//            // �ж������Ƿ�����Ӧ���ź�Ӧ��Ϊ10-19
//            for (i=0, error=0; i<10; i++ )
//            {
//                if (ack_buffer[i] != (i+10))    { error=1; break; }
//            }
//
//            if ((ack_len==10) && (error==0))    { return (1); } // ��������
//        }
//    }
    
    return (1);
}



/*===========================================================================
* ���� : main() => ���������������                                         *
* ˵�� ��ÿ1s����һ�����ݣ�ÿ�����ݳ���Ϊ10���ֽڣ���������Ϊ0-9            *
         ���շ�����(Ӧ��)�����ݳ���Ϊ10���ֽڣ���������Ϊ10-19              *
============================================================================*/
void RF_Tx(void)
{
	//System_Initial();                       // ��ʼ��ϵͳ��������

	//while (1)
	{
	    //�������ݰ���ÿ����һ�Σ�LED��˸һ��
	    //if (0 != SendFlag)                  // 1s������������
        {
            // ���ݷ��ͳɹ�(�յ����շ���Ӧ��)
            if (RF_SendPacket(SendBuffer, SEND_LENGTH))
            {
                //LED1_ON();                  // LED����������ָʾӦ��ɹ�
                //USART_Send("Ack ok\r\n", 8);
              //  GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);  // for TM V2.0
                GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);   // for TM V3.0
            }
            else
            {
                //LED1_OFF();
                //USART_Send("Ack error\r\n", 11);
               // GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1); // for TM V2.0
              //  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1); // for TM V3.0
            }
            
            //SendFlag = 0;
        }
	}
}

/*===========================================================================
* ���� ��RF_RecvHandler() => �������ݽ��մ���                               *
============================================================================*/
void RF_Rx(void)
{
    INT8U error=0, i=0, length=0, recv_buffer[65]={ 0 };

    SI446X_INT_STATUS(recv_buffer);     // ����Ƿ��յ�һ�����ݰ�

    // �յ�һ�����ݰ�����תLED
    if (recv_buffer[3] & (1<<4))
    {
        length = SI446X_READ_PACKET(recv_buffer);

        // �ж������Ƿ����󣬽��յ����ź�Ӧ��Ϊ0-9
        for (i=0, error=0; i<10; i++)
        {
            if (recv_buffer[i] != i)    { error=1; break; } // ���ݳ���
        }

        if ((length==10) && (error==0))                     // ������ȷ
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
//            LED0_TOG();                     // LED��˸������ָʾ�յ���ȷ����
//            USART_Send("Receive ok\r\n", 12);
//            DelayMs(10);
//
//            // ����Ӧ���ź�,Ӧ������Ϊ10-19
//            SI446X_SEND_PACKET(AckBuffer, ACK_LENGTH, 0, 0);
//            do
//            {
//                SI446X_INT_STATUS(recv_buffer);
//            }while (!(recv_buffer[3] & (1<<5)));    //�ȴ�������ɣ��жϲ�����
//
//            RecvCnt++;
        }

        //�ص�����ģʽ�������ȴ���һ������
        //SI446X_START_RX(0, 0, PACKET_LENGTH,8, 8, 8);
    }
}

/*
// Tx Set Temperature, increase
void TxSetTemperatureinc(void)
{
    SendBuffer[0]++;
    RF_Tx();
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0+GPIO_PIN1);
}


// Tx Set Temperature, reduce
void TxSetTemperaturreduce(void)
{
    SendBuffer[0]--;
    RF_Tx();
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0+GPIO_PIN1);
}
*/

// Tx Set Temperature
void TxSetTemperatur(void)
{
   // SendBuffer[0]--;
    SendBuffer[0] = returnSetTemperature ();
    RF_Tx();
//    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0+GPIO_PIN1);  // for TM_V2.0
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);    // for TM_V3.0
}

