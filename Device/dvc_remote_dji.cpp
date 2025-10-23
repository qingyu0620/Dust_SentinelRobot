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

#define MaxVelocity     10
#define MaxOmega        10
#define MaxReload       15

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Dji遥控初始化
 * 
 * @param huart uart句柄
 * @param callback_function 回调函数
 * @param rx_buffer_length 接受缓冲区长度
 */
void RemoteDjiDR16::Init(UART_HandleTypeDef* huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
{
    uart_init(huart, callback_function, rx_buffer_length);
}

/**
 * @brief Dbus转换函数
 * 
 * @param buffer 传入遥控数据
 */
void RemoteDjiDR16::DbusTransformation(uint8_t* buffer)
{
    // 读取数据值
    data.ch0 =  ((int16_t)buffer[0]       | ((int16_t)buffer[1] << 8)) & 0x07FF;
    data.ch1 = (((int16_t)buffer[1] >> 3) | ((int16_t)buffer[2] << 5)) & 0x07FF;
    data.ch2 = (((int16_t)buffer[2] >> 6) | ((int16_t)buffer[3] << 2)  | ((int16_t)buffer[4] << 10)) & 0x07FF;
    data.ch3 = (((int16_t)buffer[4] >> 1) | ((int16_t)buffer[5] << 7)) & 0x07FF;

    data.s1 = ((buffer[5] >> 4) & 0x000C) >> 2;
    data.s2 = ((buffer[5] >> 4) & 0x0003);

    //线性函数归一化（-1 ~ 1） * 放大系数
    output.chassis_x  = (k_nor * data.ch0 + c_nor) * MaxVelocity;
    output.chassis_y  = (k_nor * data.ch1 + c_nor) * MaxVelocity;
    output.chassis_y  = (k_pitch * data.ch1 - c_pitch);
    output.chassis_r  = (k_nor * data.ch2 + c_nor) * MaxOmega;
    output.r0 = (k_nor * data.ch3 + c_nor) * MaxReload;
    
    output.keyL = data.s1;
    output.keyR = data.s2;
}