/*===========================================================================
��ַ ��http://yhmcu.taobao.com/
���� ������  ԭ �ں͵��ӹ�����  �� �ڰ��ص��ӿƼ����޹�˾ 
�ʼ� ��yihe_liyong@126.com
�绰 ��18615799380
===========================================================================*/

#ifndef _BSP_H_
#define _BSP_H_

#include <msp430fr4133.h>
#include "si446x.h"
#include "mytypedef.h"

// SPI���Ŷ��� SCLK(PB5), MOSI(PB6), MISO(PB7)
//#define PORT_SPI        GPIOB
//#define PIN_SCLK        GPIO_Pin_5
//#define PIN_MOSI        GPIO_Pin_6
//#define PIN_MISO        GPIO_Pin_7

// LED��KEY���Ŷ��壬LED(PB0), KEY(PB1)
//#define PORT_LED0       GPIOA
//#define PIN_LED0        GPIO_Pin_2
//
//#define PORT_LED1       GPIOA
//#define PIN_LED1        GPIO_Pin_3
//
//#define PORT_KEY        GPIOC
//#define PIN_KEY         GPIO_Pin_0
//
//#define PORT_USART      GPIOC
//#define PIN_TXD         GPIO_Pin_3
//#define PIN_RXD         GPIO_Pin_2
//
//// LED����������(ON)��, (OFF)�رգ�(TOG)��ת
//#define LED0_ON()       GPIO_ResetBits(PORT_LED0, PIN_LED0)
//#define LED0_OFF()      GPIO_SetBits(PORT_LED0, PIN_LED0)
//#define LED0_TOG()      GPIO_ToggleBits(PORT_LED0, PIN_LED0)
//
//#define LED1_ON()       GPIO_ResetBits(PORT_LED1, PIN_LED1)
//#define LED1_OFF()      GPIO_SetBits(PORT_LED1, PIN_LED1)
//#define LED1_TOG()      GPIO_ToggleBits(PORT_LED1, PIN_LED1)
//
//#define KEY_READ()      GPIO_ReadInputDataBit(PORT_KEY, PIN_KEY)
//
//void SClK_Initial(void);                // ��ʼ��ϵͳʱ�ӣ�ϵͳʱ�� = 16MHZ
void GPIO_Initial(void);                // ��ʼ��ͨ��IO�˿�
//void SPI_Initial(void);                 // ��ʼ��SPI
//void TIM3_Initial(void);                // ��ʼ����ʱ��3����ʱʱ��Ϊ1ms
//void USART_Initial(void);               // ��ʼ������
//
//INT8U SPI_ExchangeByte(INT8U input);    // ͨ��SPI�������ݽ���

#endif //_BSP_H_

/*===========================================================================
-----------------------------------�ļ�����----------------------------------
===========================================================================*/
