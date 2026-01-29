/**
 * @file dvc_remote_vt02.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote_vt02.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief VT02清理数据函数
 * 
 */
void RemoteDjiVT02::ClearData()
{
    output_.mouse.mouse_x = 0;
    output_.mouse.mouse_y = 0;
    output_.mouse.mouse_z = 0;

    output_.mouse.mouse_lr.mousecode.mouse_l = REMOTE_KEY_STATUS_FREE;
    output_.mouse.mouse_lr.mousecode.mouse_r = REMOTE_KEY_STATUS_FREE;

    output_.keyboard.all = REMOTE_KEY_STATUS_FREE;
}

/**
 * @brief VT02数据处理函数
 * 
 */
void RemoteDjiVT02::DataProcess(uint8_t* buffer)
{
    /****************************   原始数据    ****************************/

    raw_data_.start_of_frame = buffer[0];
    raw_data_.data_length = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);
    raw_data_.sequence = buffer[3];
    raw_data_.crc8 = buffer[5];
    raw_data_.cmd_id = buffer[6];

    int16_t dx = (int16_t)((uint16_t)buffer[7] | ((uint16_t)buffer[8] << 8));
    int16_t dy = (int16_t)((uint16_t)buffer[9] | ((uint16_t)buffer[10] << 8));
    // int16_t dz = (int16_t)((uint16_t)buffer[11] | ((uint16_t)buffer[12] << 8));

    raw_data_.mouse_x = CLAMP(dx * 15, INT16_MIN, INT16_MAX);
    raw_data_.mouse_y = CLAMP(dy, INT16_MIN, INT16_MAX);
    // raw_data_.mouse_z = CLAMP(dz, INT16_MIN, INT16_MAX);

    raw_data_.mouse_l = buffer[13];
    raw_data_.mouse_r = buffer[14];
    raw_data_.keyboard.all = (uint16_t)buffer[15] | ((uint16_t)buffer[16] << 8);


    /****************************   键鼠数据    ****************************/

    output_.mouse.mouse_x = (int16_t)raw_data_.mouse_x;
    output_.mouse.mouse_y = -(float)raw_data_.mouse_y / (float)INT16_MAX;
    // output_.mouse.mouse_z = (int16_t)raw_data_.mouse_z;

    output_.mouse.mouse_lr.mousecode.mouse_l = raw_data_.mouse_l;
    output_.mouse.mouse_lr.mousecode.mouse_r = raw_data_.mouse_r;

    Process_Keyboard_Toggle(&output_.keyboard, raw_data_.keyboard);
}