/*===========================================================================
网址 ：http://yhmcu.taobao.com/
作者 ：李勇  原 亿和电子工作室  现 亿佰特电子科技有限公司 
邮件 ：yihe_liyong@126.com
电话 ：18615799380
===========================================================================*/

#ifndef _TX_H_
#define _TX_H_

#include <msp430fr4133.h>
#include "si446x.h"
#include "mytypedef.h"

//void RF_Tx(void);                // 初始化通用IO端口
void System_Initial(void);
void RF_Tx(void);
void RF_Rx(void);
void TxSetTemperatureinc(void);
void TxSetTemperaturreduce(void);
void TxSetTemperatur(void);

#endif //_TX_H_

/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/
