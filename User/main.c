/*
*********************************************************************************************************
*
*	ģ������ : ������ģ�顣
*	�ļ����� : main.c
*	��    �� : V1.0
*	˵    �� : ��ʵ����ҪѧϰRL-FlashFS+SD��������ۺ�ʵ�֣�
*              ʵ��Ŀ�ģ�
*                1. ѧϰRL-FlashFS+SD��������ۺ�ʵ�֣�
*              ʵ�����ݣ�
*                1. FlashFS�����ĵ���ͨ�����Զ˷������ָ������弴�ɡ�
*					printf("��ѡ�񴮿ڲ���������Լ��̴�ӡ���ּ���:\r\n");
*					printf("1 - ��ʾSD��������ʣ������\r\n");
*					printf("2 - ��ʾSD����Ŀ¼�µ��ļ�\r\n");
*					printf("3 - ��������text�ı���ʹ�ò�ͬ����д������\r\n");
*					printf("4 - ʹ��������ͬ������ȡ�ı�\r\n");
*					printf("5 - ����һ��text�ı���ָ��һ��λ�ö�����ж�д����\r\n");
*					printf("6 - ɾ���ļ��к��ļ�\r\n");
*					printf("7 - �����ļ���д�ٶ�\r\n");
*					printf("8 - д���ݵ�CSV�ļ���\r\n");
*					printf("9 - ��CSV�ļ��ж�����\r\n");
*              ע�����
*                1. ÿ��MDK�İ�װĿ¼���涼����һ��RTXԴ�룬����MDK4.XX��˵�����ʹ�õ�
*                   �Ǹ�MDK�汾�����ʹ���Ǹ�MDK�汾�����RTX������ʹ��MDK�Դ���RTX�������
*                   ʱ��������ʾ��ȷ�ĵ�����Ϣ��
*                2. ��ǰʹ�õ�RTXԴ����MDK4.74����ģ�KEIL�ٷ��Ѿ�����MDK4ϵ�еĸ����ˣ�
*                   ����汾����MDK4ϵ���������°汾�ˡ������Ҫʹ��MDK�Դ���RTX���������ʾ
*                   ��Ϣ�������ʹ��MDK4.74.
*                3. ����MDK5.XX��RTXҲ���䰲װĿ¼���棬����RTX�Ѿ�����Ϊ�����汾�����ˣ�
*                   ����һ��ȫ�µ����ֽ�CMSIS-RTOS RTX��ARM�ٷ���RTX�Ļ����ϸ�RTX������һ���װ��
*                4. ʹ��FlashFS�Ļ����Ͳ�����ʹ��MicroLib�����鿴Retarget.c�ļ���ͷ��˵����
*                5. FlashFSֻ��MDK�Ŀ⣬û��IAR��GCC��
*                6. ��ʵ���Ƽ�ʹ�ô�������SecureCRT��Ҫ�����ڴ�ӡЧ�������롣��������
*                   V4��������������С�
*                7. ��ؽ��༭��������������TAB����Ϊ4���Ķ����ļ���Ҫ��������ʾ�����롣
*
*	�޸ļ�¼ :
*		�汾��    ����         ����            ˵��
*       V1.0    2015-09-10   Eric2013    1. ST�̼��⵽V3.6.1�汾
*                                        2. BSP������V1.2
*                                        3. RL-FlashFS�汾V4.74
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"		


/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
extern uint8_t isFree;
int main (void) 
{	
	/* ��ʼ������ */
    isFree = 0;
	bsp_Init();	
	while(1)
	{
		DemoFlashFS();
	}
}
