/**
 * @file dvc_remote_dji.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote_dji.h"
#include "stdio.h"

/* Private macros ------------------------------------------------------------*/

#define k               1.0f / 660.0f
#define c               -256.0f / 165.0f
#define MaxVelocity     10
#define MaxOmega        10

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Dbus转换函数
 * 
 * @param buffer 传入遥控数据
 */
void RemoteDjiDR16::DbusTransformation(uint8_t* buffer)
{
    // 读取数据值
    remotedata.ch0 =  ((int16_t)buffer[0]       | ((int16_t)buffer[1] << 8)) & 0x07FF;
    remotedata.ch1 = (((int16_t)buffer[1] >> 3) | ((int16_t)buffer[2] << 5)) & 0x07FF;
    remotedata.ch2 = (((int16_t)buffer[2] >> 6) | ((int16_t)buffer[3] << 2)  | ((int16_t)buffer[4] << 10)) & 0x07FF;
    remotedata.ch3 = (((int16_t)buffer[4] >> 1) | ((int16_t)buffer[5] << 7)) & 0x07FF;

    remotedata.s1 = ((buffer[5] >> 4) & 0x000C) >> 2;
    remotedata.s2 = ((buffer[5] >> 4) & 0x0003);

    //线性函数归一化（-1 ~ 1） * 放大系数
    remotedata.x = (k * remotedata.ch0 + c) * MaxVelocity;
    remotedata.y = (k * remotedata.ch1 + c) * MaxVelocity;
    remotedata.r = (k * remotedata.ch2 + c) * MaxOmega;
}