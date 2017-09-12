#include "includes.h"		

#define BUFFERSIZE 50

uint8_t g_uart3_timeout; //��⴮��3�������ݳ�ʱ��ȫ�ֱ���

RECVDATA_T t_Uartbuf;//�ݴ洮������ʹ��

//����Ϊ��־λ����
uint8_t buffer1IsFull = FALSE;//buffer1�������Ƿ�����
uint8_t buffer2IsFull = FALSE;//buffer2�������Ƿ�����
uint8_t	bufferSelect = 1;//ѡ���Ǹ���������ȡֵΪ1����2

uint8_t flagofUartRecv = 0;//�Ӵ��ڽ��մ��������ݱ�־λ
uint8_t isEnterIRQ = FALSE;//�յ����ݱ�־λ//��spi�ӿڽ��յ����ݵı�־λ��
uint8_t MasterBstisRcv = FALSE; //�������㲥��������ȷ�����õı�־λ
uint8_t isReceiveAllFrame = FALSE;//���յ�������һ֡����


SLVMSG_SD s_tToSDBuffer1[BUFFERSIZE];//�洢��sd���Ľṹ�建��1
SLVMSG_SD s_tToSDBuffer2[BUFFERSIZE];//�洢��sd���Ľṹ�建��2

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

//static void Uart3_RxTimeOut(void)
//{
//    g_uart3_timeout = 1;
//}
//void Uart3Callback_ReciveNew(uint8_t _byte)
//{
//    /*
//        3.5���ַ���ʱ������ֻ������RTUģʽ���棬��ΪRTUģʽû�п�ʼ���ͽ�������
//        �������ݰ�֮��ֻ�ܿ�ʱ���������֣�Modbus�����ڲ�ͬ�Ĳ������£����ʱ���ǲ�һ���ģ�
//        ���Ծ���3.5���ַ���ʱ�䣬�����ʸߣ����ʱ������С�������ʵͣ����ʱ������Ӧ�ʹ�

//        4800  = 7.297ms
//        9600  = 3.646ms
//        19200  = 1.771ms
//        38400  = 0.885ms
//    */
//    uint32_t timeout;

//    g_uart3_timeout = 0; 

////    timeout = 35000000 / 115200;            /* ���㳬ʱʱ�䣬��λus 35000000*/

////  printf("%x\t",_byte);
////  Ӳ����ʱ�жϣ���ʱ����us��ʹ�ö�ʱ��2���ڼ����ճ�ʱ
//    timeout = 3000;//1����100�λش������10ms�����ݰ�ʱ��Ϊ33�ֽ�*8λ/115200=2.3ms
//    //��ʱ3msû�����ݣ���Ϊһ�����ݰ�����    
//    
//    bsp_StartHardTimer(3, timeout, (void *)Uart3_RxTimeOut);

//    if (p_tUart3.RxCount < S_RX_BUF_SIZE)
//    {
//        p_tUart3.RxBuf[p_tUart3.RxCount++] = _byte;
//    }
//}

int main (void) 
{		
    uint8_t read1;
    uint8_t read2;
    uint8_t read3;
    uint8_t t_count;
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
        //�ж��Ƿ���յ�lora�Ŀ�ʼ�ͽ����ź�
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
            }//end of ���յ���ʼ�ź�
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
            }//end of ���յ������ź�

        }//end of �ж��Ƿ��յ�lora�ź�
                
//        if (g_uart3_timeout == 0)// û�г�ʱ���������ա���Ҫ���� g_tUartx.RxCount
//        {
//            continue; 
//        }
//                
//        g_uart3_timeout = 0; // ��ʱ���־

        
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
                            isReceiveAllFrame = TRUE;
                        }
                        else
                        {
                            continue;
                        }
                    }
                }
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }
             
            if(isReceiveAllFrame == TRUE)
            {
                
                memcpy(&stcAcc, &t_Uartbuf.RxBuf[0], 8);
                memcpy(&stcGyro, &t_Uartbuf.RxBuf[11], 8);
                memcpy(&stcMag, &t_Uartbuf.RxBuf[22], 8);

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
                memset(t_Uartbuf.RxBuf, 0, sizeof(t_Uartbuf.RxBuf)); 
                isReceiveAllFrame = FALSE;
            }                
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



