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

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void RemoteDjiDR16::DbusTransformation(uint8_t* buffer)
{
    remotedata.ch0 = ((int16_t)buffer[0] | ((int16_t)buffer[1] << 8)) & 0x07FF;
    remotedata.ch1 = (((int16_t)buffer[1] >> 3) | ((int16_t)buffer[2] << 5)) & 0x07FF;
    remotedata.ch2 = (((int16_t)buffer[2] >> 6) | ((int16_t)buffer[3] << 2) | ((int16_t)buffer[4] << 10)) & 0x07FF;
    remotedata.ch3 = (((int16_t)buffer[4] >> 1) | ((int16_t)buffer[5]<<7)) & 0x07FF;

    remotedata.s1 = ((buffer[5] >> 4) & 0x000C) >> 2;
    remotedata.s2 = ((buffer[5] >> 4) & 0x0003);

    printf("%d,%d,%d,%d,%d,%d\n", remotedata.ch0, remotedata.ch1, remotedata.ch2, remotedata.ch3, remotedata.s1, remotedata.s2);
}