/********************************************************************************************************
*
*   模块名称 : 串口中断+FIFO驱动模块
*   文件名称 : Uart_fifo.h
*   版    本 : V1.0
*   说    明 : 头文件
*
*********************************************************************************************************/

#ifndef _BSP_USART_FIFO_H_
#define _BSP_USART_FIFO_H_
#include "bsp.h"
/*
    如果需要更改串口对应的管脚，请自行修改 Uart_fifo.c文件中的 static void UART_InitHardPara(void)函数
*/

/* 定义使能的串口, 0 表示不使能（不增加代码大小）， 1表示使能 */
/*
    USBPC电路板串口分配：
    【串口1】 CH340T芯片第1路。调试阶段作为信息输出。
        PA9/USART1_TX
        PA10/USART1_RX

    【串口2】SX1278 模块
        PA2/USART2_TX
        PA3/USART2_RX

    【串口3】 GPS 通信（在USBPC中无法使用）
        PB10/USART3_TX
        PB11/USART3_RX
        PB2/BOOT1/RS485_TX_EN

    【串口4】 --- 不做串口用。
    【串口5】 --- 不做串口用。
*/
#define UART1_FIFO_EN   1
#define UART2_FIFO_EN   1
#define UART3_FIFO_EN   1
#define UART4_FIFO_EN   0
#define UART5_FIFO_EN   0
#define UART6_FIFO_EN	0

/* RS485芯片发送使能GPIO, PB2 */
#define RCC_RS485_TXEN   RCC_APB2Periph_GPIOB
#define PORT_RS485_TXEN  GPIOB
#define PIN_RS485_TXEN   GPIO_Pin_2

#define RS485_RX_EN()   PORT_RS485_TXEN->BRR = PIN_RS485_TXEN
#define RS485_TX_EN()   PORT_RS485_TXEN->BSRR = PIN_RS485_TXEN


/* 定义端口号 */
typedef enum
{
    COM1 = 0,   /* USART1  PA9, PA10 */
    COM2 = 1,   /* USART2, PA2, PA3 */
    COM3 = 2,   /* USART3, PB10, PB11 */
    COM4 = 3,   /* UART4, PC10, PC11 */
    COM5 = 4,   /* UART5, PC12, PD2 */
} COM_PORT_E;

/* 定义串口波特率和FIFO缓冲区大小，分为发送缓冲区和接收缓冲区, 支持全双工 */
#if UART1_FIFO_EN == 1
#define UART1_BAUD          115200
#define UART1_TX_BUF_SIZE   1*16
#define UART1_RX_BUF_SIZE   1*16
#endif

#if UART2_FIFO_EN == 1
#define UART2_BAUD          115200
#define UART2_TX_BUF_SIZE   1*16
#define UART2_RX_BUF_SIZE   1*16
#endif

#if UART3_FIFO_EN == 1
#define UART3_BAUD          230400
#define UART3_TX_BUF_SIZE   1*16
#define UART3_RX_BUF_SIZE   1*33*100*6
#endif

#if UART4_FIFO_EN == 1
#define UART4_BAUD          115200
#define UART4_TX_BUF_SIZE   1*16
#define UART4_RX_BUF_SIZE   1*16
#endif

#if UART5_FIFO_EN == 1
#define UART5_BAUD          115200
#define UART5_TX_BUF_SIZE   1*16
#define UART5_RX_BUF_SIZE   1*16
#endif

/* 串口设备结构体 */
typedef struct
{
    USART_TypeDef *uart;        /* STM32内部串口设备指针 */
    uint8_t *pTxBuf;            /* 发送缓冲区 */
    uint8_t *pRxBuf;            /* 接收缓冲区 */
    uint16_t usTxBufSize;       /* 发送缓冲区大小 */
    uint16_t usRxBufSize;       /* 接收缓冲区大小 */
    __IO uint16_t usTxWrite;            /* 发送缓冲区写指针 */
    __IO uint16_t usTxRead;         /* 发送缓冲区读指针 */
    __IO uint16_t usTxCount;            /* 等待发送的数据个数 */

    __IO uint16_t usRxWrite;            /* 接收缓冲区写指针 */
    __IO uint16_t usRxRead;         /* 接收缓冲区读指针 */
    __IO uint16_t usRxCount;            /* 还未读取的新数据个数 */

    void (*SendBefore)(void);   /* 开始发送之前的回调函数指针（主要用于RS485切换到发送模式） */
    void (*SendOver)(void);     /* 发送完毕的回调函数指针（主要用于RS485将发送模式切换为接收模式） */
    void (*ReciveNew)(uint8_t _byte);   /* 串口收到数据的回调函数指针 */
} UART_T;

void UART_InitALL(void);
void COMx_SendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);   //_ucPort串口号，_ucaBuf串口发送缓冲区，_usLen数据长度
void COMx_SendChar(COM_PORT_E _ucPort, uint8_t _ucByte);    //_ucPort串口号，_ucByte串口发送字节数据
uint8_t COMx_GetChar(COM_PORT_E _ucPort, uint8_t *_pByte);  //_ucPort串口号，_pByte串口接收缓冲区

void COMx_ClearTxFifo(COM_PORT_E _ucPort);
void COMx_ClearRxFifo(COM_PORT_E _ucPort);

//void RS485_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen);
//void RS485_SendStr(char *_pBuf);

void UART3_SetBaud(uint32_t _baud);
void UART1_SetBaud(uint32_t _baud);
void UART2_SetBaud(uint32_t _baud);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
