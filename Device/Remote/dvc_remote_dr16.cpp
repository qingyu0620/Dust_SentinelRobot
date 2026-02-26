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

#include "dvc_remote_dr16.h"

/* Private macros ------------------------------------------------------------*/

#define K_NORM      1.f / 660.f
#define C_NORM      -256.f / 165.f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief DjiDR16清理数据函数
 * 
 */
void RemoteDjiDR16::ClearData()
{
    output_.remote.pitch = K_PITCH * 1024 + C_PITCH;
    output_.remote.chassis_x = output_.remote.chassis_y = output_.remote.rotation = 1024;
    output_.remote.switch_l = output_.remote.switch_r = 3;

    output_.keyboard.all = 0;
    output_.mouse.mouse_x = output_.mouse.mouse_y = 0;
    output_.mouse.mouse_lr.mousecode.mouse_l = output_.mouse.mouse_lr.mousecode.mouse_r = 0;
}

/**
 * @brief DjiDR16数据处理函数
 * 
 */
void RemoteDjiDR16::DataProcess(uint8_t* buffer)
{
    /****************************   原始数据    ****************************/

    raw_data_.rc.ch0 =  ((int16_t)buffer[0]        | ((int16_t)buffer[1] << 8)) & 0x07FF;
    raw_data_.rc.ch1 = (((int16_t)buffer[1] >> 3)  | ((int16_t)buffer[2] << 5)) & 0x07FF;
    raw_data_.rc.ch2 = (((int16_t)buffer[2] >> 6)  | ((int16_t)buffer[3] << 2)  | ((int16_t)buffer[4] << 10)) &  0x07FF;
    raw_data_.rc.ch3 = (((int16_t)buffer[4] >> 1)  | ((int16_t)buffer[5] << 7)) & 0x07FF;

    raw_data_.rc.s1 = ((buffer[5] >> 4) & 0x000C) >> 2;
    raw_data_.rc.s2 = ((buffer[5] >> 4) & 0x0003);

    int16_t dx = ((int16_t)buffer[6]) | ((int16_t)buffer[7] << 8);
    int16_t dy = ((int16_t)buffer[8]) | ((int16_t)buffer[9] << 8);

    raw_data_.mouse.x = CLAMP(dx * 30, INT16_MIN, INT16_MAX);
    raw_data_.mouse.y = CLAMP(dy * 2, INT16_MIN, INT16_MAX);

    raw_data_.mouse.pl = buffer[12];
    raw_data_.mouse.pr = buffer[13];

    raw_data_.keyboard.all = (int16_t)buffer[14];

    /****************************   遥控数据    ****************************/

    // 上板数据
    output_.remote.pitch = K_PITCH * raw_data_.rc.ch3 + C_PITCH;
    output_.remote.pitch = CLAMP(output_.remote.pitch, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

    // 下板数据
    output_.remote.chassis_x  = raw_data_.rc.ch1;
    output_.remote.chassis_y  = raw_data_.rc.ch0;
    output_.remote.rotation   = raw_data_.rc.ch2;

    // 通用数据
    output_.remote.switch_l = raw_data_.rc.s1;
    output_.remote.switch_r = raw_data_.rc.s2;

    /****************************   键鼠数据    ****************************/

    // 鼠标数据
    output_.mouse.mouse_x = (1683 + 1320 * ((int16_t)raw_data_.mouse.x - 32767) / 65535);
    output_.mouse.mouse_y += (float)raw_data_.mouse.y / 32767.f;
    output_.mouse.mouse_y = CLAMP(output_.mouse.mouse_y, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

    output_.mouse.mouse_lr.mousecode.mouse_l = raw_data_.mouse.pl;
    output_.mouse.mouse_lr.mousecode.mouse_r = raw_data_.mouse.pr;

    // 键盘数据
    Process_Keyboard_Toggle(&output_.keyboard, raw_data_.keyboard);
}
