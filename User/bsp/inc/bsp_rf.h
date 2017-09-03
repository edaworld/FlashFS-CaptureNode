#ifndef __BSP_RF_H__
#define	__BSP_RF_H__
#include "bsp.h"
#define RF_SX1278

extern const char *rfName;
extern u16	iSend, iRev;

extern u8	sendBuf[64];
extern u8	revBuf[128];

void SPI2_Init(void);
u8 RFSendData(u8 *buf, u8 size);
u8 RFRevData(u8 *buf);

void RFGPIOInit(void);
void RFRxMode(void);
void RFInit(void);
//u8 rfContinueSend(void);
#endif
