/*
*********************************************************************************************************
*
*	模块名称 : FlashFS文件系统演示模块。
*	文件名称 : demo_sd_flashfs.c
*	版    本 : V1.0
*	说    明 : 该例程移植FlashFS文件系统，主要学习FlashFS的裸机综合操作。
*   注意事项 : 1. FlashFS不支持操作中文文件名的文件，仅支持在使用短文件名库FSN_CM3.lib列出8.3格式中文名。
*                 但是使用fopen函数操作中文文件名的文件时会出错。
*              2. FlashFS只有MDK的库，没有IAR和GCC。
*
*********************************************************************************************************
*/
#include "includes.h"

#define BUFFERSIZE 600
extern uint8_t g_uart1_timeout; //检测串口1接收数据超时的全局变量，在bsp_uartpro.c文件中声明
extern uint8_t g_uart2_timeout; //检测串口2接收数据超时的全局变量，在bsp_uartpro.c文件中声明
extern uint8_t g_uart3_timeout; //检测串口2接收数据超时的全局变量，在bsp_uartpro.c文件中声明

extern RECVDATA_T g_tUart1; //初始化从串口1，调试输出接口，在bsp_uartfifo.c和bsp_uartpro.c文件中都有声明
extern RECVDATA_T g_tUart2; //初始化从串口2，目前未使用，在bsp_uartfifo.c和bsp_uartpro.c文件中都有声明
extern RECVDATA_T g_tUart3; //初始化从串口3，从MPU9250模块接收数据，在bsp_uartfifo.c和bsp_uartpro.c文件中都有声明

//以下为标志位定义
uint8_t buffer1IsFull = FALSE;//buffer1缓冲区是否已满
uint8_t buffer2IsFull = FALSE;//buffer2缓冲区是否已满
uint8_t	bufferSelect = 1;//选择那个缓冲区，取值为1或者2

//接收串口数据的计数器
uint8_t flagofUartRecv = 0;//从串口接收传感器数据标志位
//从spi接口接收到数据的标志位
uint8_t isEnterIRQ = FALSE;//收到数据标志位，
//从主机解析正确的数据包标志位
uint8_t MasterBstisRcv = FALSE; //从主机广播包解析正确，设置的标志位

SLVMSG_SD s_tToSDBuffer1[BUFFERSIZE];//存储到sd卡的结构体缓冲1
SLVMSG_SD s_tToSDBuffer2[BUFFERSIZE];//存储到sd卡的结构体缓冲2

//接收到的传感器数据
ACC_U accofReceive;//接收到的加速度数据
ANG_U angofReceive;//接收到的角度数据
MAG_U magofReceive;//接收到的磁力计数据

struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle 	stcAngle;
struct SMag     stcMag;

//static void SetJY901(void);
char *filename = "M0:\\test\\test.txt";
/*********************************************************************************************************
*	函 数 名: DemoFlashFS
*	功能说明: FlashFS件系统演示主程序
*	形    参: 无
*	返 回 值: 无
**********************************************************************************************************/
uint8_t isFree;
static uint8_t recvdatbuffer[32];   //接收数据缓冲区
static uint16_t countofbuf1;
static uint16_t	countofbuf2;
static uint32_t cnt;
FILE *fout;
char *filename;
uint32_t bw;
uint8_t result;
static uint8_t savetimes;
FINFO info;
uint8_t mountcount;
uint8_t unmountcount;
void DemoFlashFS(void)
{
//    SetJY901();//设置九轴陀螺仪   
    mountcount = 3;
    while (finit("M0:") != 0) 
    {
        if (!(mountcount--)) 
        {
            isFree = 0;
            return;
        }
    }    
    isFree = 1;
    
    if(ffind (filename, &info) == 0)//查找是否有filename文件，如有，删除
    { 
        fdelete (filename);
    }
    
	fout = fopen (filename, "w"); //打开文件夹test中的文件，如果没有子文件夹和txt文件会自动创建
    
	if (fout != NULL) 
	{
        isFree = 1;
		printf("打开文件成功\r\n");
	}
	else
	{
        isFree = 0;
		printf("打开文件失败\r\n");
	}       
    while(1)
    {
        if (GPIO_ReadInputDataBit(GPIOC, RF_IRQ_PIN)) //SPI的SX1278，接收到数据时，IRQ引脚为高
		{
			isEnterIRQ = TRUE;           
            if (isEnterIRQ  == TRUE)//主函数中检测到中断引脚为高，表示接收到数据后，置位该标志位
            {
                RFM96_LoRaRxPacket(recvdatbuffer);
                RFRxMode();
                if ((recvdatbuffer[0] == '%') && (recvdatbuffer[8] == '&') && (recvdatbuffer[1] == 0xAA) && (recvdatbuffer[2] == 0x55))
                {
                    MasterBstisRcv = TRUE;   //设置接收到的主机包标志位
                    isEnterIRQ = FALSE;  
                    bsp_LedOn(1); 
                    isFree = 2;
                }
                if ((recvdatbuffer[0] == '%') && (recvdatbuffer[8] == '&') && (recvdatbuffer[1] == 0x55) && (recvdatbuffer[2] == 0xAA))
                {
                    MasterBstisRcv = FALSE;   //设置接收到的主机包标志位
                    isEnterIRQ = FALSE;   
                    bsp_LedOff(1); 
                    fclose(fout);
                    
                    unmountcount = 3;
                    while (funinit("M0:") != 0) 
                    {
                        if (!(unmountcount--)) 
                        {
                            isFree = 0;
                            return;
                        }
                    }    
                    isFree = 1;                    
                }
            }
		}//end of judge sx1278 is received
        
 
        if (g_uart3_timeout == 0)
        {
            continue; // 没有超时，继续接收。不要清零 g_tUart1.RxCount
        }
        
        if (g_tUart3.RxCount < 33)    // 接收到的数据小于11个字节就认为错误
        {
            continue;
        }
        
        g_uart3_timeout = 0; // 超时清标志
        g_tUart3.RxCount = 0; // 必须清零计数器，方便下次帧同步
        
        if(MasterBstisRcv == TRUE)
        {
            if ((g_tUart3.RxBuf[0] == 0x55) && (g_tUart3.RxBuf[11] == 0x55) && (g_tUart3.RxBuf[22] == 0x55)) //检测数据包头是否正确
            {
                if (g_tUart3.RxBuf[1] == 0x51)
                {
                    memcpy(&stcAcc, &g_tUart3.RxBuf[2], 8);
                    memset(&g_tUart3.RxBuf[0], 0, 11);
                    flagofUartRecv ++;
                }
                if (g_tUart3.RxBuf[12] == 0x52)
                {
                    memcpy(&stcGyro, &g_tUart3.RxBuf[13], 8);
                    memset(&g_tUart3.RxBuf[11], 0, 11);
                    flagofUartRecv ++;
                }
                if (g_tUart3.RxBuf[23] == 0x54)
                {
                    memcpy(&stcMag, &g_tUart3.RxBuf[24], 8);
                    memset(&g_tUart3.RxBuf[22], 0, 11);
                    flagofUartRecv ++;
                }
                if (flagofUartRecv == 3)
                {
                    flagofUartRecv = 0;
                    if (bufferSelect == 1)
                    {
                        s_tToSDBuffer1[countofbuf1].acc = stcAcc;
                        s_tToSDBuffer1[countofbuf1].ang = stcGyro;
                        s_tToSDBuffer1[countofbuf1].mag = stcMag;
                        s_tToSDBuffer1[countofbuf1].times = cnt;
                        countofbuf1 ++;
                        cnt++;
                    }
                    if (bufferSelect == 2)
                    {
                        s_tToSDBuffer2[countofbuf2].acc = stcAcc;
                        s_tToSDBuffer2[countofbuf2].ang = stcGyro;
                        s_tToSDBuffer2[countofbuf2].mag = stcMag;
                        s_tToSDBuffer2[countofbuf2].times = cnt;
                        countofbuf2 ++;
                        cnt++;
                    }
                    if (countofbuf1 == BUFFERSIZE)
                    {
                        countofbuf1 = 0;
                        bufferSelect = 2;
                        buffer1IsFull = TRUE;
                    }
                    if (countofbuf2 == BUFFERSIZE)
                    {
                        countofbuf2 = 0;
                        bufferSelect = 1;
                        buffer2IsFull = TRUE;
                    }
                }
            }
        }
        memset(g_tUart3.RxBuf, 0, sizeof(g_tUart3.RxBuf));
        
        if(MasterBstisRcv == TRUE)
        {    
            if (buffer1IsFull == TRUE)
            {
                fseek (fout, 0L, SEEK_END);                
                bw = fwrite (s_tToSDBuffer1, sizeof(SLVMSG_SD), sizeof(s_tToSDBuffer1)/sizeof(SLVMSG_SD), fout);
                memset(s_tToSDBuffer1, 0, sizeof(s_tToSDBuffer1));
                fseek (fout, 0L, SEEK_END);
                fflush(fout);
                buffer1IsFull = FALSE;
            }
            if (buffer2IsFull == TRUE)
            {
                fseek (fout, 0L, SEEK_END); 
                bw = fwrite (s_tToSDBuffer2, sizeof(SLVMSG_SD), sizeof(s_tToSDBuffer2)/sizeof(SLVMSG_SD), fout);
                memset(s_tToSDBuffer2, 0, sizeof(s_tToSDBuffer2));
                fseek (fout, 0L, SEEK_END);
                fflush(fout);                
                buffer2IsFull = FALSE;
            }
            if(++savetimes == 20)//保存10次以后，close一下，然后再打开，保证能够写入！
            {
                savetimes = 0;
                fclose(fout);
                fout = fopen(filename,"a");
            }            
        }   
    }//endof while(1)
}
#if 0
void SetJY901(void)
{
    //设置代码
    uint8_t setBaudRateCode[5] = {0xFF, 0xAA, 0x04, 0x06, 0x00};//设置波特率为115200
    uint8_t setFeedBackSpeed[5] = {0xFF, 0xAA, 0x03, 0x09, 0x00};//设置回传速率为100Hz
    uint8_t setFeedBackContent[5] = {0xFF, 0xAA, 0x02, 0x16, 0x00};//设置输出加速度，角速度，磁场数据包
    uint8_t saveConfiguration[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};//设置保存当前设置数
    COMx_SendBuf(COM3, setFeedBackContent, 5);
	bsp_DelayMS(50);
	COMx_SendBuf(COM3, setFeedBackSpeed, 5);
	bsp_DelayMS(50);
	COMx_SendBuf(COM3, setBaudRateCode, 5);
	bsp_DelayMS(50);
	COMx_SendBuf(COM3, saveConfiguration, 5);
	bsp_DelayMS(50);    
}
#endif
