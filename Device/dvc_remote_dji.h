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

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct RemoteParameter
{
    float k, c;
};

struct RemoteDjiData
{
    uint16_t ch0, ch1, ch2, ch3;
    uint8_t s1, s2;
};

struct RemoteOutput
{
    uint8_t keyL, keyR;
    float x, y, r;      // x, y, r 采用右手系
    float r0;           // r0为保留位，对应左摇杆上下
};

class RemoteDjiDR16
{
public:
    RemoteOutput output;
    void DbusTransformation(uint8_t* buffer);
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