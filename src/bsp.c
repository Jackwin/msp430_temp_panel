/*===========================================================================
* 网址 ：http://www.cdebyte.com/   http://yhmcu.taobao.com/                 *
* 作者 ：李勇  原 亿和电子工作室  现 亿佰特电子科技有限公司                 * 
* 邮件 ：yihe_liyong@126.com                                                *
* 电话 ：18615799380                                                        *
============================================================================*/

#include "bsp.h"

/*===========================================================================
* 函数 ：SClK_Initial() => 初始化系统时钟，系统时钟 = 4MHZ                  *
============================================================================*/
//void SClK_Initial(void)
//{
//	CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv4); // 4M
//}

/*===========================================================================
* 函数 ：GPIO_Initial() => 初始化通用IO端口                                 *
============================================================================*/
void GPIO_Initial(void)
{
    // 配置LED引脚和KEY引脚 KEY(PB1), LED(PB0)
//    GPIO_Init(PORT_KEY, PIN_KEY, GPIO_Mode_In_PU_No_IT);
//
//    GPIO_Init(PORT_LED0, PIN_LED0, GPIO_Mode_Out_PP_High_Slow);
//    GPIO_SetBits(PORT_LED0, PIN_LED0);
//
//    GPIO_Init(PORT_LED1, PIN_LED1, GPIO_Mode_Out_PP_High_Slow);
//    GPIO_SetBits(PORT_LED1, PIN_LED1);
    

    // 配置SI44XX相关控制引脚 SDN(PD0), CSN(PB0), GIO0(PB1), GDO1(PB2)
    GPIO_setAsInputPinWithPullUpResistor(PORT_SI_GIO0, PIN_SI_GIO0);
    GPIO_setAsInputPinWithPullUpResistor(PORT_SI_GIO1, PIN_SI_GIO1);
    
    GPIO_setAsOutputPin(PORT_SI_CSN, PIN_SI_CSN);
    GPIO_setOutputHighOnPin(PORT_SI_CSN, PIN_SI_CSN);
    
    GPIO_setAsOutputPin(PORT_SI_SDN, PIN_SI_SDN);
    GPIO_setOutputHighOnPin(PORT_SI_SDN, PIN_SI_SDN);
}

/*===========================================================================
* 函数 ：SPI_Initial() => 初始化SPI                                         *
============================================================================*/
//void SPI_Initial(void)
//{
//	CLK_PeripheralClockConfig(CLK_Peripheral_SPI, ENABLE);
//
//	SPI_DeInit();
//
//	// 配置SPI相关参数,2分频（8MHZ）
//	SPI_Init(SPI_FirstBit_MSB, SPI_BaudRatePrescaler_2,
//             SPI_Mode_Master, SPI_CPOL_Low, SPI_CPHA_1Edge,
//             SPI_Direction_2Lines_FullDuplex, SPI_NSS_Soft);
//
//	SPI_Cmd(ENABLE);
//
//	// SPI相关IO口配置
//	GPIO_Init(PORT_SPI, PIN_MISO, GPIO_Mode_In_PU_No_IT);       // MISO (PB7)
//	GPIO_Init(PORT_SPI, PIN_SCLK, GPIO_Mode_Out_PP_High_Fast);  // SCLK (PB5)
//	GPIO_Init(PORT_SPI, PIN_MOSI, GPIO_Mode_Out_PP_High_Fast);  // MOSI (PB6)
//}

/*=============================================================================
*Function:  USART_Initial() => 初始化串口
=============================================================================*/
//void USART_Initial(void)
//{
//    GPIO_Init(PORT_USART, PIN_RXD, GPIO_Mode_In_FL_No_IT);      // RXD
//    GPIO_Init(PORT_USART, PIN_TXD, GPIO_Mode_Out_OD_HiZ_Fast);  // TXD
//
//    CLK_PeripheralClockConfig(CLK_Peripheral_USART, ENABLE);
//
//    USART_Init((uint32_t)9600, USART_WordLength_8D, USART_StopBits_1,
//                USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Rx | USART_Mode_Tx));
//
////    USART_ITConfig(USART_IT_TXE, ENABLE);
//    USART_ITConfig(USART_IT_RXNE, ENABLE);
//
//    USART_ClearITPendingBit();
//
//    USART_Cmd(ENABLE);
//
//}

/*===========================================================================
* 函数 ：TIM3_Initial() => 初始化定时器3，定时时间为1ms                     *
============================================================================*/
//void TIM3_Initial(void)
//{
//    TIM3_DeInit();
//
//    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);
//
//    // 配置Timer3相关参数，时钟为4/4 = 1MHZ，定时时间 = 1000/1000000 = 1ms
//    TIM3_TimeBaseInit(TIM3_Prescaler_4, TIM3_CounterMode_Up, 1000);
//    TIM3_ITConfig(TIM3_IT_Update, ENABLE);
//
//    TIM3_Cmd(ENABLE);
//}

/*===========================================================================
* 函数 ：SPI_ExchangeByte() => 通过SPI进行数据交换                          * 
* 输入 ：需要写入SPI的值                                                    * 
* 输出 ：通过SPI读出的值                                                    * 
============================================================================*/
//INT8U SPI_ExchangeByte(INT8U input)
//{
//    SPI_SendData(input);
//	while (RESET == SPI_GetFlagStatus(SPI_FLAG_TXE));   // 等待数据传输完成
//	while (RESET == SPI_GetFlagStatus(SPI_FLAG_RXNE));  // 等待数据接收完成
//	return (SPI_ReceiveData());
//}

/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/
