#include "includes.h"		

#define BUFFERSIZE 500

//����Ϊ��־λ����
uint8_t buffer1IsFull = FALSE;//buffer1�������Ƿ�����
uint8_t buffer2IsFull = FALSE;//buffer2�������Ƿ�����
uint8_t	bufferSelect = 1;//ѡ���Ǹ���������ȡֵΪ1����2

uint8_t flagofUartRecv = 0;//�Ӵ��ڽ��մ��������ݱ�־λ
uint8_t isEnterIRQ = FALSE;//�յ����ݱ�־λ//��spi�ӿڽ��յ����ݵı�־λ��
uint8_t MasterBstisRcv = FALSE; //�������㲥��������ȷ�����õı�־λ

extern uint8_t g_uart1_timeout; //��⴮��1�������ݳ�ʱ��ȫ�ֱ�������bsp_uartpro.c�ļ�������
extern uint8_t g_uart2_timeout; //��⴮��2�������ݳ�ʱ��ȫ�ֱ�������bsp_uartpro.c�ļ�������
extern uint8_t g_uart3_timeout; //��⴮��2�������ݳ�ʱ��ȫ�ֱ�������bsp_uartpro.c�ļ�������
extern RECVDATA_T p_tUart1; //��ʼ���Ӵ���1����������ӿڣ���bsp_uartfifo.c��bsp_uartpro.c�ļ��ж�������
extern RECVDATA_T p_tUart2; //��ʼ���Ӵ���2��Ŀǰδʹ�ã���bsp_uartfifo.c��bsp_uartpro.c�ļ��ж�������
extern RECVDATA_T p_tUart3; //��ʼ���Ӵ���3����MPU9250ģ��������ݣ���bsp_uartfifo.c��bsp_uartpro.c�ļ��ж�������

RECVDATA_T t_Uartbuf;//�ݴ洮������ʹ��

extern uint8_t TPCTaskNum; //������������main.c�б���ʼ����bsp_tpc.c������

uint8_t isReceiveAllFrame = FALSE;//���յ�������һ֡����


SLVMSG_SD s_tToSDBuffer1[BUFFERSIZE];//�洢��sd���Ľṹ�建��1
//SLVMSG_SD s_tToSDBuffer2[BUFFERSIZE];//�洢��sd���Ľṹ�建��2

//ACC_U accofReceive;//���յ��ļ��ٶ�����
//ANG_U angofReceive;//���յ��ĽǶ�����
//MAG_U magofReceive;//���յ��Ĵ���������

struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle 	stcAngle;
struct SMag     stcMag;

//char *filename = "M0:\\test\\test.txt";

uint8_t isFree;
static uint8_t recvdatbuffer[32];   //�������ݻ�����
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

extern uint8_t isFree;//ָʾ�Ʊ�־λ

const uint8_t WriteText[] = {"�人�������������޹�˾\r\n2015-09-06\r\nwww.armfly.com\r\nWWW.ARMFLY.COM"};

static void SetJY901(void)
{
    //���ô���
    uint8_t setBaudRate_1[5] = {0xFF, 0xAA, 0x04, 0x06, 0x00};//���ò�����Ϊ 115200
    uint8_t setBaudRate_2[5] = {0xFF, 0xAA, 0x04, 0x07, 0x00};//���ò�����Ϊ230400    
    uint8_t setFeedBackSpeed[5] = {0xFF, 0xAA, 0x03, 0x09, 0x00};//���ûش�����Ϊ100Hz
    uint8_t setFeedBackContent[5] = {0xFF, 0xAA, 0x02, 0x16, 0x00};//����������ٶȣ����ٶȣ��ų����ݰ�
    uint8_t saveConfiguration[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};//���ñ��浱ǰ������
    uint8_t resetConfiguration[5] = {0xFF, 0xAA, 0x00, 0x01, 0x00};//�ָ�Ĭ�����ò�����    
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
	//���������ʱ����ע�ⵥ�������иı��������ԵĴ���
	{ 0, 0, 10, 1, Task_RecvfromLora }, // ��̬���񣬴����SPI�ӿڵ�NRF���յ��������񣬵ȴ���ʼ��ͬ���ź�
	{ 0, 0, 5, 1, Task_WriteToSD }, // ��̬����д������SD��
	{ 0, 0, 1, 1, Task_RecvfromUart }, // ��̬����,ͨ�����ڴ�MPU9250������������
//	{ 0, 0, 10, 50, Task_LEDDisplay }, // ��̬����LED��˸����ʱ��Ƭ���Ｔ��ִ��    
};

int main (void) 
{		

    isFree = 1;
	bsp_Init();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
	TaskInit(); //��ʼ������,���ǻ�ȡ�������������������bsp_idle�е���task_processʵ��
//    SetJY901();
    while(1)
    {
		if (GPIO_ReadInputDataBit(GPIOC, RF_IRQ_PIN)) //SPI��SX1278�����յ�����ʱ��IRQ����Ϊ��
		{
			isEnterIRQ = TRUE;
		}
		bsp_Idle(); // CPU����ִ�еĲ����� ִ��TPCProcess������ʵʩ�������        
    }
 
}



/*********************************************************************************************************
*   �� �� ��: Task_RecvfromLora
*   ����˵��: �����SPI�ӿڵ�Lora���յ���������
*********************************************************************************************************/
void Task_RecvfromLora(void)
{
	if (isEnterIRQ  == TRUE)//�������м�⵽�ж�����Ϊ�ߣ���ʾ���յ����ݺ���λ�ñ�־λ
	{
        RFM96_LoRaRxPacket(recvdatbuffer);
        RFRxMode();
        if ((recvdatbuffer[0] == '%') && (recvdatbuffer[8] == '&') && (recvdatbuffer[1] == 0xAA) && (recvdatbuffer[2] == 0x55))
        {
            MasterBstisRcv = TRUE;   //���ý��յ�����������־λ
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
                if(ffind (pch, &info) == 0)//�����Ƿ���filename�ļ������У�ɾ��
                { 
                    fdelete (pch);
                    
                    fout = fopen (pch, "w"); //���ļ���test�е��ļ������û�����ļ��к�txt�ļ����Զ�����
                    inum ++;
                }
                else
                {
                    fout = fopen (pch, "w"); //���ļ���test�е��ļ������û�����ļ��к�txt�ļ����Զ�����
                    if (fout != NULL) 
                    {
                        isFree = 2;
                        printf("���ļ��ɹ�\r\n");
                    }
                    else
                    {
                        isFree = 0;
                        printf("���ļ�ʧ��\r\n");                            
                        return;
                    }
                    inum++;
                }            
            }while(fout == NULL);
//            fwrite (WriteText, sizeof(uint8_t), sizeof(WriteText)/sizeof(uint8_t), fout);                 
        }//end of ���յ���ʼ�ź�
        if ((recvdatbuffer[0] == '%') && (recvdatbuffer[8] == '&') && (recvdatbuffer[1] == 0x55) && (recvdatbuffer[2] == 0xAA))
        {
            MasterBstisRcv = FALSE;   //���ý��յ�����������־λ
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
        }//end of ���յ������ź�
		isEnterIRQ = FALSE;
	}
}
/*********************************************************************************************************
*   �� �� ��: Task_WriteToSD
*   ����˵��: �������ݰ�����������
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
        if(++savetimes == 20)//����10���Ժ�closeһ�£�Ȼ���ٴ򿪣���֤�ܹ�д�룡
        {
            savetimes = 0;
            fclose(fout);
            fout = fopen(pch,"a");
        }        
    }
}
/*********************************************************************************************************
*   �� �� ��: Task_RecvfromUart
*   ����˵��: �����uart1�ӿڽ��յ���CC2541���͹�������������
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
*   �� �� ��: TaskInit
*   ����˵��: �����ʼ��
*********************************************************************************************************/
void TaskInit(void)
{
	TPCTaskNum = (sizeof(TaskComps) / sizeof(TaskComps[0])); // ��ȡ������
}
/*********************************************************************************************************
*   �� �� ��: Task_LEDDisplay
*   ����˵��: LED��˸����
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
