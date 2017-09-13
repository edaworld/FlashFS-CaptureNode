#include "includes.h"		

#define BUFFERSIZE 500

//以下为标志位定义
uint8_t buffer1IsFull = FALSE;//buffer1缓冲区是否已满
uint8_t buffer2IsFull = FALSE;//buffer2缓冲区是否已满
uint8_t	bufferSelect = 1;//选择那个缓冲区，取值为1或者2

uint8_t flagofUartRecv = 0;//从串口接收传感器数据标志位
uint8_t isEnterIRQ = FALSE;//收到数据标志位//从spi接口接收到数据的标志位，
uint8_t MasterBstisRcv = FALSE; //从主机广播包解析正确，设置的标志位

extern uint8_t g_uart1_timeout; //检测串口1接收数据超时的全局变量，在bsp_uartpro.c文件中声明
extern uint8_t g_uart2_timeout; //检测串口2接收数据超时的全局变量，在bsp_uartpro.c文件中声明
extern uint8_t g_uart3_timeout; //检测串口2接收数据超时的全局变量，在bsp_uartpro.c文件中声明
extern RECVDATA_T p_tUart1; //初始化从串口1，调试输出接口，在bsp_uartfifo.c和bsp_uartpro.c文件中都有声明
extern RECVDATA_T p_tUart2; //初始化从串口2，目前未使用，在bsp_uartfifo.c和bsp_uartpro.c文件中都有声明
extern RECVDATA_T p_tUart3; //初始化从串口3，从MPU9250模块接收数据，在bsp_uartfifo.c和bsp_uartpro.c文件中都有声明

RECVDATA_T t_Uartbuf;//暂存串口数据使用

extern uint8_t TPCTaskNum; //任务数量，在main.c中被初始化，bsp_tpc.c中声明

uint8_t isReceiveAllFrame = FALSE;//接收到完整的一帧数据


SLVMSG_SD s_tToSDBuffer1[BUFFERSIZE];//存储到sd卡的结构体缓冲1
//SLVMSG_SD s_tToSDBuffer2[BUFFERSIZE];//存储到sd卡的结构体缓冲2

//ACC_U accofReceive;//接收到的加速度数据
//ANG_U angofReceive;//接收到的角度数据
//MAG_U magofReceive;//接收到的磁力计数据

struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle 	stcAngle;
struct SMag     stcMag;

//char *filename = "M0:\\test\\test.txt";

uint8_t isFree;
static uint8_t recvdatbuffer[32];   //接收数据缓冲区
static uint16_t countofbuf1;
static uint16_t	countofbuf2;
static uint32_t cnt;

FILE *fout;
uint32_t bw;
uint8_t result;
FINFO info;
static uint8_t savetimes;
uint8_t mountcount;
uint8_t unmountcount;
char pch[25];
static volatile uint16_t inum=0;

extern uint8_t isFree;//指示灯标志位

const uint8_t WriteText[] = {"武汉安富莱电子有限公司\r\n2015-09-06\r\nwww.armfly.com\r\nWWW.ARMFLY.COM"};

static void SetJY901(void)
{
    //设置代码
    uint8_t setBaudRate_1[5] = {0xFF, 0xAA, 0x04, 0x06, 0x00};//设置波特率为 115200
    uint8_t setBaudRate_2[5] = {0xFF, 0xAA, 0x04, 0x07, 0x00};//设置波特率为230400    
    uint8_t setFeedBackSpeed[5] = {0xFF, 0xAA, 0x03, 0x09, 0x00};//设置回传速率为100Hz
    uint8_t setFeedBackContent[5] = {0xFF, 0xAA, 0x02, 0x16, 0x00};//设置输出加速度，角速度，磁场数据包
    uint8_t saveConfiguration[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};//设置保存当前设置数
    uint8_t resetConfiguration[5] = {0xFF, 0xAA, 0x00, 0x01, 0x00};//恢复默认配置并保存    
    COMx_SendBuf(COM3, setFeedBackContent, 5);
	bsp_DelayMS(50);
	COMx_SendBuf(COM3, setFeedBackSpeed, 5);
	bsp_DelayMS(50);
//	COMx_SendBuf(COM3, setBaudRate_1, 5);
//	bsp_DelayMS(50);
	COMx_SendBuf(COM3, setBaudRate_2, 5);
	bsp_DelayMS(50);    
	COMx_SendBuf(COM3, saveConfiguration, 5);
	bsp_DelayMS(50);
//	COMx_SendBuf(COM3, resetConfiguration, 5);
//	bsp_DelayMS(50);    
}

TPC_TASK TaskComps[3] =
{
	//添加新任务时，请注意单个任务中改变任务属性的代码
	{ 0, 0, 10, 1, Task_RecvfromLora }, // 静态任务，处理从SPI接口的NRF接收的数据任务，等待开始的同步信号
	{ 0, 0, 5, 1, Task_WriteToSD }, // 静态任务，写数据至SD卡
	{ 0, 0, 1, 1, Task_RecvfromUart }, // 静态任务,通过串口从MPU9250接收数据任务
//	{ 0, 0, 10, 50, Task_LEDDisplay }, // 静态任务，LED闪烁任务，时间片到达即可执行    
};

int main (void) 
{		

    isFree = 1;
	bsp_Init();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
	TaskInit(); //初始化任务,仅是获取任务数量，任务调度在bsp_idle中调用task_process实现
//    SetJY901();
    while(1)
    {
		if (GPIO_ReadInputDataBit(GPIOC, RF_IRQ_PIN)) //SPI的SX1278，接收到数据时，IRQ引脚为高
		{
			isEnterIRQ = TRUE;
		}
		bsp_Idle(); // CPU空闲执行的操作， 执行TPCProcess函数，实施任务调度        
    }
 
}



/*********************************************************************************************************
*   函 数 名: Task_RecvfromLora
*   功能说明: 处理从SPI接口的Lora接收的数据任务
*********************************************************************************************************/
void Task_RecvfromLora(void)
{
	if (isEnterIRQ  == TRUE)//主函数中检测到中断引脚为高，表示接收到数据后，置位该标志位
	{
        RFM96_LoRaRxPacket(recvdatbuffer);
        RFRxMode();
        if ((recvdatbuffer[0] == '%') && (recvdatbuffer[8] == '&') && (recvdatbuffer[1] == 0xAA) && (recvdatbuffer[2] == 0x55))
        {
            MasterBstisRcv = TRUE;   //设置接收到的主机包标志位
            mountcount = 3;
            while (finit("M0:") != 0) 
            {
                if (!(mountcount--)) 
                {
                    isFree = 0;
                    return;
                }
            }    
            isFree = 2;
            fout = NULL;
            do
            {
                sprintf((char *)pch,"M0:\\DATA\\TRECORD%d.TXT",inum);
                if(inum > 500) return;
                if(ffind (pch, &info) == 0)//查找是否有filename文件，如有，删除
                { 
                    fdelete (pch);
                    
                    fout = fopen (pch, "w"); //打开文件夹test中的文件，如果没有子文件夹和txt文件会自动创建
                    inum ++;
                }
                else
                {
                    fout = fopen (pch, "w"); //打开文件夹test中的文件，如果没有子文件夹和txt文件会自动创建
                    if (fout != NULL) 
                    {
                        isFree = 2;
                        printf("打开文件成功\r\n");
                    }
                    else
                    {
                        isFree = 0;
                        printf("打开文件失败\r\n");                            
                        return;
                    }
                    inum++;
                }            
            }while(fout == NULL);
//            fwrite (WriteText, sizeof(uint8_t), sizeof(WriteText)/sizeof(uint8_t), fout);                 
        }//end of 接收到开始信号
        if ((recvdatbuffer[0] == '%') && (recvdatbuffer[8] == '&') && (recvdatbuffer[1] == 0x55) && (recvdatbuffer[2] == 0xAA))
        {
            MasterBstisRcv = FALSE;   //设置接收到的主机包标志位
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
        }//end of 接收到结束信号
		isEnterIRQ = FALSE;
	}
}
/*********************************************************************************************************
*   函 数 名: Task_WriteToSD
*   功能说明: 发送数据包至主机任务
*********************************************************************************************************/
void Task_WriteToSD(void)
{
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
        if(++savetimes == 20)//保存10次以后，close一下，然后再打开，保证能够写入！
        {
            savetimes = 0;
            fclose(fout);
            fout = fopen(pch,"a");
        }        
    }
}
/*********************************************************************************************************
*   函 数 名: Task_RecvfromUart
*   功能说明: 处理从uart1接口接收到的CC2541发送过来的数据任务
*********************************************************************************************************/
static uint32_t cnt;
uint8_t read1;
uint8_t read2;
uint8_t read3;
uint8_t t_count;
void Task_RecvfromUart(void)
{
    if(MasterBstisRcv == TRUE)
    {
        read1 = 0x00;            
        if(COMx_GetChar(COM3,&read1) == 1)
        {
            if(read1 == 0x55)
            {
                read2 = 0x00;  
                if(COMx_GetChar(COM3,&read2) == 1)
                {
                    if(read2 == 0x51)
                    {
                        do
                        {
                            if(COMx_GetChar(COM3,&read3) == 1)
                            {
                                t_Uartbuf.RxBuf[t_count++] = read3;
                            }
                        }while(t_count<31);
                        t_count = 0;
                        cnt++;
                        memcpy(&stcAcc, &t_Uartbuf.RxBuf[0], 8);                        
                        memcpy(&stcGyro, &t_Uartbuf.RxBuf[11], 8);                        
                        memcpy(&stcMag, &t_Uartbuf.RxBuf[22], 8);
                        s_tToSDBuffer1[countofbuf1].acc = stcAcc;
                        s_tToSDBuffer1[countofbuf1].ang = stcGyro;
                        s_tToSDBuffer1[countofbuf1].mag = stcMag;
                        s_tToSDBuffer1[countofbuf1].times = cnt; 
                        countofbuf1 ++;
                        if (countofbuf1 == BUFFERSIZE)
                        {
                            countofbuf1 = 0;
                            buffer1IsFull = TRUE;
                        }
                        memset(t_Uartbuf.RxBuf, 0, sizeof(t_Uartbuf.RxBuf));                            
                    }
                }
            }
        }
    }        
}
/*********************************************************************************************************
*   函 数 名: TaskInit
*   功能说明: 任务初始化
*********************************************************************************************************/
void TaskInit(void)
{
	TPCTaskNum = (sizeof(TaskComps) / sizeof(TaskComps[0])); // 获取任务数
}
/*********************************************************************************************************
*   函 数 名: Task_LEDDisplay
*   功能说明: LED闪烁代码
*********************************************************************************************************/
void Task_LEDDisplay(void)
{
//    if(MasterBstisRcv == FALSE)
//    {
//        TaskComps[0].ItvTime = 50;
//    }
//    if(MasterBstisRcv == TRUE)
//    {
//        TaskComps[0].ItvTime = 1000;
//    }
}
