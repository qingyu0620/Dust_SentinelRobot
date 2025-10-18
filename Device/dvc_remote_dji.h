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

struct RemoteDjiData
{
    uint16_t ch0, ch1, ch2, ch3;
    uint8_t s1, s2;
    float x, y, r;
};

class RemoteDjiDR16
{
public:
    RemoteDjiData remotedata;
    void DbusTransformation(uint8_t* buffer);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif