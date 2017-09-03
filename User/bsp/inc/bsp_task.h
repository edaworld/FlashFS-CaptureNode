#ifndef  __TASK__H__
#define  __TASK__H__
#include "bsp.h"
#pragma anon_unions             //使用匿名结构体

/********************************************************************************************************
* 宏定义
********************************************************************************************************/

typedef union
{
	uint8_t msg[8];
	struct
	{
		uint8_t head; //包头
		uint8_t devID; //设备id
		uint8_t Heartdata; //心率值
		uint8_t HrtPowerdata; //心率带电量
		uint8_t BatPowerdata; //电池电量
		uint8_t tail; //包尾
	};
} SLVMSG_T; //从机信息结构体

typedef union 
{
    uint8_t msg[6];		
    struct
    {
        uint8_t AxL;//low byte of x acc
        uint8_t AxH;//high byte of x acc
        uint8_t AyL;//low byte of y acc
        uint8_t AyH;//high byte of y acc
        uint8_t AzL;//low byte of z acc
        uint8_t AzH;//high byte of z acc
     };
}ACC_U;

typedef union 
{
    uint8_t msg[6]; 	
    struct
    {
        uint8_t wxL;//low byte of x ang vel
        uint8_t wxH;//high byte of x ang vel
        uint8_t wyL;//low byte of y ang vel
        uint8_t wyH;//high byte of y ang vel
        uint8_t wzL;//low byte of z ang vel
        uint8_t wzH;//high byte of z ang vel
    };
}ANG_U;

typedef union
{
    uint8_t msg[6]; 	
    struct
    {
        uint8_t HxL;//low byte of x mag
        uint8_t HxH;//high byte of x mag
        uint8_t HyL;//low byte of y mag
        uint8_t HyH;//high byte of y mag
        uint8_t HzL;//low byte of z mag
        uint8_t HzH;//high byte of z mag
     };
}MAG_U;


struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
struct SMag
{
	short h[3];
	short T;
};

typedef struct
{
    uint32_t times;
//    ACC_U acc;
//    ANG_U ang;
//    MAG_U mag;
    struct SAcc acc;
    struct SGyro ang;
    struct SMag mag;
}SLVMSG_SD;

/********************************************************************************************************
* 变量定义
********************************************************************************************************/

/********************************************************************************************************
* 内部函数
********************************************************************************************************/

/********************************************************************************************************
* 全局函数
********************************************************************************************************/

/*******************************************************************************************************/
#endif

