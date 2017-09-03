/*********************************************************************************************************
*   模块名称 : RF模块(For STM32F1XX)
*   文件名称 : bsp_rf.c
*   说    明 : 
*********************************************************************************************************/
#include "bsp.h"

const char *rfName = "SX1278";

u16	iSend, iRev;    //无线发送和接收计数
u8	sendBuf[64];    //发送缓冲区
u8	revBuf[128];    //接收缓冲区

void RFGPIOInit(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;   
	/* Configure PA8 RF_CE as Output push-pull -------------------------------*/
    //此引脚在原理图中是NREST引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PB12 RF_Csn as Output push-pull -------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PC6 RF_IRQ as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PB11 RF_SDN as Output push-pull -------------------------------*/
    //此引脚没有用上
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

//射频模块初始化
void RFInit(void)
{
	SPI2_Init();
	RFM96_LoRaEntryRx(); //进入接收模式
}

//射频模块进入接收模式
void RFRxMode(void)
{
	RFM96_LoRaEntryRx(); //进入接收模式
}

//射频模块接收数据
u8 RFRevData(u8 *buf)
{
	u8 length = 0;
	if (GPIO_ReadInputDataBit(GPIOC, RF_IRQ_PIN)) //收到数据高电平中断
	{
		length = RFM96_LoRaRxPacket(revBuf);
		RFRxMode();
	}
	if (length > 0) iRev++; //接收数据个数
	return (length);
}
//射频模块发射数据
u8 RFSendData(u8 *buf, u8 size)
{
	int ret = 0;
	ret = RFM96_LoRaEntryTx(size); //返回发送字节数
	ret = RFM96_LoRaTxPacket(buf, size); //返回发送字节数

	bsp_DelayMS(5);
	RFRxMode(); //进入接收模式
    
	if (ret > 0) 
        iSend++;    //发送数据个数
	return (ret); //成功后大于0的值
}


