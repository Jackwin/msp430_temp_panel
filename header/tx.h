/*===========================================================================
��ַ ��http://yhmcu.taobao.com/
���� ������  ԭ �ں͵��ӹ�����  �� �ڰ��ص��ӿƼ����޹�˾ 
�ʼ� ��yihe_liyong@126.com
�绰 ��18615799380
===========================================================================*/

#ifndef _TX_H_
#define _TX_H_

#include <msp430fr4133.h>
#include "si446x.h"
#include "mytypedef.h"

//void RF_Tx(void);                // ��ʼ��ͨ��IO�˿�
void System_Initial(void);
void RF_Tx(void);
void RF_Rx(void);
void TxSetTemperatureinc(void);
void TxSetTemperaturreduce(void);
void TxSetTemperatur(void);

#endif //_TX_H_

/*===========================================================================
-----------------------------------�ļ�����----------------------------------
===========================================================================*/
