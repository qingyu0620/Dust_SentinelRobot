/**
 * @file dvc_remote_dji.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __DVC_REMOTE_DJI_H__
#define __DVC_REMOTE_DJI_H__
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "bsp_uart.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

enum RemoteSwitchStatus
{
    Switch_UP    = (uint8_t)1,
    Switch_MID   = (uint8_t)3,
    Switch_DOWN  = (uint8_t)2,
};

struct RemoteDjiData
{
    uint16_t ch0, ch1, ch2, ch3;
    uint8_t s1, s2;
};

struct RemoteOutput
{
    uint8_t SwitchL, SwitchR;
    float chassis_x, chassis_y, chassis_r;      // x, y, r 采用右手系
    float shoot_speed;
    float gimbal_pitch;                         
    float gimbal_yaw;                           
};

class RemoteDjiDR16
{
public:
    RemoteOutput output;
    void Init(UART_HandleTypeDef* huart, Uart_Callback callback_function, uint16_t rx_buffer_length);
    void DataProcess(uint8_t* buffer);
protected:
    RemoteDjiData data;

    // 归一化线性转换参数
    float k_nor = 1.0f / 660.0f;
    float c_nor = -256.0f / 165.0f;

    // pitch线性转换参数（当前为-53度为摇杆最低，-48为摇杆最高）
    float k_pitch = 1.f / 264.f;
    float c_pitch = 3589.f / 66.f;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif