#ifndef BSP_SLAVEMSG_H
#define BSP_SLAVEMSG_H

#include "stdint.h"


#define S_RX_BUF_SIZE       50
#define S_TX_BUF_SIZE       50

typedef struct
{
    uint8_t RxBuf[S_RX_BUF_SIZE];
	uint8_t RxCount;
//	uint8_t RxStatus;
//	uint8_t RxNewFlag;

	uint8_t TxBuf[S_TX_BUF_SIZE];
	uint8_t TxCount;
} RECVDATA_T;


void Uart1Callback_ReciveNew(uint8_t _byte);
void Uart2Callback_ReciveNew(uint8_t _byte);
void Uart3Callback_ReciveNew(uint8_t _byte);
#endif
