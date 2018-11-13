/*===========================================================================
* 网址 ：http://www.cdebyte.com/   http://yhmcu.taobao.com/                 *
* 作者 ：李勇  原 亿和电子工作室  现 亿佰特电子科技有限公司                 * 
* 邮件 ：yihe_liyong@126.com                                                *
* 电话 ：18615799380                                                        *
============================================================================*/

#include "tx.h"
#include "bsp.h"
#include "TempSensorMode.h"

// 常量定义
#define TX              1       // 发送模式
#define RX              0       // 接收模式

#define SEND_GAP        1000    // 每间隔1s发送一次数据
#define RECV_TIMEOUT    800     // 接收超时

#define ACK_LENGTH      10      // 应答信号长度        
#define SEND_LENGTH     10      // 发送数据每包的长度

INT8U   Cnt1ms = 0;             // 1ms计数变量，每1ms加一 
INT8U   SendFlag = 0;           // =1，发送无线数据，=0不处理

INT16U  SendTime = 1;           // 计数数据发送间隔时间
INT16U  RecvWaitTime = 0;       // 接收等待时间                
INT16U  SendCnt = 0;            // 计数发送的数据包数                

// 需要发送的数据  
INT8U   SendBuffer[SEND_LENGTH] = {25, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

// 串口相关变量
INT8U   COM_TxNeed = 0;
INT8U   COM_RxCounter = 0;
INT8U   COM_TxCounter = 0;
INT8U   COM_RxBuffer[65] = { 0 };
INT8U   COM_TxBuffer[65] = { 0 };

//#define USRAT_SendByte()    USART_SendData8(COM_TxBuffer[COM_TxCounter++])
//#define USRAT_RecvByte()    COM_RxBuffer[COM_RxCounter++]=USART_ReceiveData8()

/*===========================================================================
* 函数 : DelayMs() => 延时函数(ms级)                                        *
* 输入 ：x, 需要延时多少(0-255)                                             *
============================================================================*/
//void DelayMs(INT8U x)
//{
//    Cnt1ms = 0;
//    while (Cnt1ms <= x);
//}

/*===========================================================================
* 函数 ：TIM3_1MS_ISR() => 定时器3服务函数, 定时时间基准为1ms               *
============================================================================*/
//void TIM3_1MS_ISR(void)
//{
//    Cnt1ms++;
//
//    if (0 != RecvWaitTime)      { RecvWaitTime--; }     // 数据接收计时
//
//    if (0 != SendTime)      // 1ms时间到，置位SendFlag标志，主函数查询发送数据
//    {
//        if (--SendTime == 0)    { SendTime = SEND_GAP; SendFlag = 1; }
//    }
//}

/*=============================================================================
* 函数 : USART_Send() => 通过串口发送数据                                     *
* 输入 ：buff, 待发送数据     size， 发送长度                                *
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
* 函数:  USART_RX_Interrupt() => 串口接收中断                                 *
=============================================================================*/
//void USART_RX_Interrupt(void)
//{
//    USRAT_RecvByte();
//}

/*=============================================================================
* 函数:  USART_TX_Interrupt() =>串口发送中断
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
* 函数 ：MCU_Initial() => 初始化CPU所有硬件                                 *
* 说明 ：关于所有硬件的初始化操作，已经被建成C库，见bsp.c文件               *
============================================================================*/
void MCU_Initial(void)
{
    //SClK_Initial();         // 初始化系统时钟，16M
    GPIO_Initial();         // 初始化GPIO                  
    //TIM3_Initial();         // 初始化定时器3，基准1ms
    //USART_Initial();        // 初始化串口
    //SPI_Initial();          // 初始化SPI

    //enableInterrupts();     // 打开总中断
}

/*===========================================================================
* 函数 ：RF_Initial() => 初始化RF芯片                                       *
* 输入 ：mode, =0,接收模式， else,发送模式                                  *
* 说明 ：SI44XX的操作，已经被建成C库，见SI446X.c文件， 提供SPI和CSN操作，	*
         即可调用其内部所有函数用户无需再关心SI44XX的寄存器操作问题。       *
============================================================================*/
void RF_Initial(INT8U mode)
{
	SI446X_RESET();         // SI446X 模块复位
    SI446X_CONFIG_INIT();   // 寄存器初始化，寄存器来自WDS配置的头文件
    SI446X_SET_POWER(0x4F); // 将输出功率配置为最大
    SI446X_START_RX(0, 0, PACKET_LENGTH, 8, 8, 8);  // 进入接收模式         
}

/*===========================================================================
* 函数: System_Initial() => 初始化系统所有外设                              *
============================================================================*/
void System_Initial(void)
{
    MCU_Initial();      // 初始化CPU所有硬件
    RF_Initial(TX);     // 初始化无线芯片,发送模式           
}

/*===========================================================================
* 函数 : BSP_RF_SendPacket() => 无线发送数据函数                            *
* 输入 ：Sendbuffer指向待发送的数据，length发送数据长度                     *
* 输出 ：0，发送失败；else，发送成功                                        *
============================================================================*/
INT8U RF_SendPacket(INT8U *Sendbuffer, INT8U length)
{
//    INT8U ack_flag = 0;         // =1,接收到应答信号，=0不处理
//    INT8U error = 0, i=0, ack_len=0, ack_buffer[65]={ 0 };
    INT8U TxBuffer[100];
    
    SI446X_SEND_PACKET(Sendbuffer, length, 0, 0);   // 发送数据    
    do
    {
        SI446X_INT_STATUS(TxBuffer);
    }while (!(TxBuffer[3] & (1<<5)));               // 等待发射完成
    
    //LED0_TOG();
    //USART_Send("Transmit ok\r\n", 13);
    
//    SI446X_START_RX(0, 0, PACKET_LENGTH, 8, 8, 8);  // 进入接收模式，等待应答
//    RecvWaitTime = RECV_TIMEOUT;                    // 等待应答超时限制为800ms
//
//    while (0 != RecvWaitTime)
//    {
//        SI446X_INT_STATUS(ack_buffer);              // 检测是否收到数据包
//
//        if (ack_buffer[3] & (1<<4))                 // 如果为1，说明收到数据包
//        {
//            ack_len = SI446X_READ_PACKET(ack_buffer);   // 读取收到的数据
//
//            // 判断数据是否有误，应答信号应该为10-19
//            for (i=0, error=0; i<10; i++ )
//            {
//                if (ack_buffer[i] != (i+10))    { error=1; break; }
//            }
//
//            if ((ack_len==10) && (error==0))    { return (1); } // 数据无误
//        }
//    }
    
    return (1);
}



/*===========================================================================
* 函数 : main() => 主函数，程序入口                                         *
* 说明 ：每1s发送一包数据，每包数据长度为10个字节，数据内容为0-9            *
         接收方反馈(应答)的数据长度为10个字节，数据内容为10-19              *
============================================================================*/
void RF_Tx(void)
{
	//System_Initial();                       // 初始化系统所有外设

	//while (1)
	{
	    //发送数据包，每发送一次，LED闪烁一次
	    //if (0 != SendFlag)                  // 1s到，发送数据
        {
            // 数据发送成功(收到接收方的应答)
            if (RF_SendPacket(SendBuffer, SEND_LENGTH))
            {
                //LED1_ON();                  // LED点亮，用于指示应答成功
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
* 函数 ：RF_RecvHandler() => 无线数据接收处理                               *
============================================================================*/
void RF_Rx(void)
{
    INT8U error=0, i=0, length=0, recv_buffer[65]={ 0 };

    SI446X_INT_STATUS(recv_buffer);     // 检测是否收到一个数据包

    // 收到一个数据包，翻转LED
    if (recv_buffer[3] & (1<<4))
    {
        length = SI446X_READ_PACKET(recv_buffer);

        // 判断数据是否有误，接收到的信号应该为0-9
        for (i=0, error=0; i<10; i++)
        {
            if (recv_buffer[i] != i)    { error=1; break; } // 数据出错
        }

        if ((length==10) && (error==0))                     // 数据正确
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
//            LED0_TOG();                     // LED闪烁，用于指示收到正确数据
//            USART_Send("Receive ok\r\n", 12);
//            DelayMs(10);
//
//            // 返回应答信号,应答数据为10-19
//            SI446X_SEND_PACKET(AckBuffer, ACK_LENGTH, 0, 0);
//            do
//            {
//                SI446X_INT_STATUS(recv_buffer);
//            }while (!(recv_buffer[3] & (1<<5)));    //等待发射完成（中断产生）
//
//            RecvCnt++;
        }

        //回到接收模式，继续等待下一包数据
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

