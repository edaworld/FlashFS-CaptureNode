
#include "includes.h"		

#define BUFFERSIZE 800
extern uint8_t g_uart1_timeout; //��⴮��1�������ݳ�ʱ��ȫ�ֱ�������bsp_uartpro.c�ļ�������
extern uint8_t g_uart2_timeout; //��⴮��2�������ݳ�ʱ��ȫ�ֱ�������bsp_uartpro.c�ļ�������
extern uint8_t g_uart3_timeout; //��⴮��2�������ݳ�ʱ��ȫ�ֱ�������bsp_uartpro.c�ļ�������

extern RECVDATA_T p_tUart1; //��ʼ���Ӵ���1����������ӿڣ���bsp_uartfifo.c��bsp_uartpro.c�ļ��ж�������
extern RECVDATA_T p_tUart2; //��ʼ���Ӵ���2��Ŀǰδʹ�ã���bsp_uartfifo.c��bsp_uartpro.c�ļ��ж�������
extern RECVDATA_T p_tUart3; //��ʼ���Ӵ���3����MPU9250ģ��������ݣ���bsp_uartfifo.c��bsp_uartpro.c�ļ��ж�������

//����Ϊ��־λ����
uint8_t buffer1IsFull = FALSE;//buffer1�������Ƿ�����
uint8_t buffer2IsFull = FALSE;//buffer2�������Ƿ�����
uint8_t	bufferSelect = 1;//ѡ���Ǹ���������ȡֵΪ1����2

//���մ������ݵļ�����
uint8_t flagofUartRecv = 0;//�Ӵ��ڽ��մ��������ݱ�־λ
//��spi�ӿڽ��յ����ݵı�־λ
uint8_t isEnterIRQ = FALSE;//�յ����ݱ�־λ��
//������������ȷ�����ݰ���־λ
uint8_t MasterBstisRcv = FALSE; //�������㲥��������ȷ�����õı�־λ

SLVMSG_SD s_tToSDBuffer1[BUFFERSIZE];//�洢��sd���Ľṹ�建��1
SLVMSG_SD s_tToSDBuffer2[BUFFERSIZE];//�洢��sd���Ľṹ�建��2

//���յ��Ĵ���������
ACC_U accofReceive;//���յ��ļ��ٶ�����
ANG_U angofReceive;//���յ��ĽǶ�����
MAG_U magofReceive;//���յ��Ĵ���������

struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle 	stcAngle;
struct SMag     stcMag;

static void SetJY901(void);
char *filename = "M0:\\test\\test.txt";
/*********************************************************************************************************
*	�� �� ��: DemoFlashFS
*	����˵��: FlashFS��ϵͳ��ʾ������
*	��    ��: ��
*	�� �� ֵ: ��
**********************************************************************************************************/
uint8_t isFree;
static uint8_t recvdatbuffer[32];   //�������ݻ�����
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

extern uint8_t isFree;//ָʾ�Ʊ�־λ

char pch[25];
static volatile uint16_t inum=0;

const uint8_t WriteText[] = {"�人�������������޹�˾\r\n2015-09-06\r\nwww.armfly.com\r\nWWW.ARMFLY.COM"};
int main (void) 
{	
    isFree = 0;
	bsp_Init();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);    
    while(1)
    {
        
        if(isEnterIRQ == FALSE)
        {
            isFree =1;
        }
        if(isEnterIRQ == TRUE)
        {
            isFree = 2;
        }
        
        if (GPIO_ReadInputDataBit(GPIOC, RF_IRQ_PIN)) //SPI��SX1278�����յ�����ʱ��IRQ����Ϊ��
        {
            RFM96_LoRaRxPacket(recvdatbuffer);
            RFRxMode();
            if ((recvdatbuffer[0] == '%') && (recvdatbuffer[8] == '&') && (recvdatbuffer[1] == 0xAA) && (recvdatbuffer[2] == 0x55))
            {
                MasterBstisRcv = TRUE;   //���ý��յ�����������־λ
                bsp_LedOn(1);
                isEnterIRQ = TRUE;
                mountcount = 3;
                while (finit("M0:") != 0) 
                {
                    if (!(mountcount--)) 
                    {
                        isFree = 0;
                        return 0;
                    }
                }    
                isFree = 2;
                fout = NULL;
                do
                {
                    sprintf((char *)pch,"M0:\\DATA\\TRECORD%d.TXT",inum);
                    if(inum > 500) return 0;
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
                            isFree = 1;
                            printf("���ļ��ɹ�\r\n");
                        }
                        else
                        {
                            isFree = 0;
                            printf("���ļ�ʧ��\r\n");                            
                            return 0;
                        }
                        inum++;
                    }            
                }while(fout == NULL);
//                fwrite (WriteText, sizeof(uint8_t), sizeof(WriteText)/sizeof(uint8_t), fout);                 
            }
            if ((recvdatbuffer[0] == '%') && (recvdatbuffer[8] == '&') && (recvdatbuffer[1] == 0x55) && (recvdatbuffer[2] == 0xAA))
            {
                MasterBstisRcv = FALSE;   //���ý��յ�����������־λ
                isEnterIRQ = FALSE;
                fclose(fout);
                bsp_LedOff(1);
                unmountcount = 3;
                while (funinit("M0:") != 0) 
                {
                    if (!(unmountcount--)) 
                    {
                        isFree = 0;
                        return 0;
                    }
                }    
                
                isFree = 1;                    
            }

        }//end of judge sx1278 is received
        

        
        if (g_uart3_timeout == 0)// û�г�ʱ���������ա���Ҫ���� g_tUartx.RxCount
        {
            continue; 
        }
        

        
        
        if (p_tUart3.RxCount < 33)    // ���յ�������С��11���ֽھ���Ϊ����
        {
            continue;
        }
        
//            if((p_tUart3.RxBuf[0] != 0x55) || (p_tUart3.RxBuf[11] != 0x55) || (p_tUart3.RxBuf[22] != 0x55))//��һ�����ݰ����ԣ��������µȴ����ݰ�ͷ
//            {
//                p_tUart3.RxCount = 0;
//                continue;
//            } 
//            else
//            {
//                cnt++;
//            }
        
        g_uart3_timeout = 0; // ��ʱ���־
        p_tUart3.RxCount = 0; // ��������������������´�֡ͬ��
        
        if(MasterBstisRcv == TRUE)
        {
            if ((p_tUart3.RxBuf[0] == 0x55) && (p_tUart3.RxBuf[11] == 0x55) && (p_tUart3.RxBuf[22] == 0x55)) //������ݰ�ͷ�Ƿ���ȷ
            {
                if (p_tUart3.RxBuf[1] == 0x51)
                {
                    memcpy(&stcAcc, &p_tUart3.RxBuf[2], 8);
                    memset(&p_tUart3.RxBuf[0], 0, 11);
                    flagofUartRecv ++;
                }
                if (p_tUart3.RxBuf[12] == 0x52)
                {
                    memcpy(&stcGyro, &p_tUart3.RxBuf[13], 8);
                    memset(&p_tUart3.RxBuf[11], 0, 11);
                    flagofUartRecv ++;
                }
                if (p_tUart3.RxBuf[23] == 0x54)
                {
                    memcpy(&stcMag, &p_tUart3.RxBuf[24], 8);
                    memset(&p_tUart3.RxBuf[22], 0, 11);
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
                    }
                    if (bufferSelect == 2)
                    {
                        s_tToSDBuffer2[countofbuf2].acc = stcAcc;
                        s_tToSDBuffer2[countofbuf2].ang = stcGyro;
                        s_tToSDBuffer2[countofbuf2].mag = stcMag;
                        s_tToSDBuffer2[countofbuf2].times = cnt;
                        countofbuf2 ++;
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
            memset(p_tUart3.RxBuf, 0, sizeof(p_tUart3.RxBuf));            
        }

        
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
            if(++savetimes == 20)//����10���Ժ�closeһ�£�Ȼ���ٴ򿪣���֤�ܹ�д�룡
            {
                savetimes = 0;
                fclose(fout);
                fout = fopen(pch,"a");
            }            
        }   
    
    }

}

#if 1
void SetJY901(void)
{
    //���ô���
    uint8_t setBaudRateCode[5] = {0xFF, 0xAA, 0x04, 0x06, 0x00};//���ò�����Ϊ115200
    uint8_t setFeedBackSpeed[5] = {0xFF, 0xAA, 0x03, 0x09, 0x00};//���ûش�����Ϊ100Hz
    uint8_t setFeedBackContent[5] = {0xFF, 0xAA, 0x02, 0x16, 0x00};//����������ٶȣ����ٶȣ��ų����ݰ�
    uint8_t saveConfiguration[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};//���ñ��浱ǰ������
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

