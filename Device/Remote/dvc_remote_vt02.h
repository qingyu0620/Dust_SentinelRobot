/**
 * @file dvc_remote_vt02.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __DVC_REMOTE_DJI_VT02_H__
#define __DVC_REMOTE_DJI_VT02_H__

/* Includes ------------------------------------------------------------------*/

#include "dvc_remote.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief VT02原始数据结构体
 * 
 */
struct RmoteVT02RawData
{
    uint8_t start_of_frame = 0xA5;
    uint16_t data_length;
    uint8_t sequence;
    uint8_t crc8;
    uint16_t cmd_id;

    int32_t mouse_x;
    int32_t mouse_y;
    int32_t mouse_z;

    uint8_t mouse_l;
    uint8_t mouse_r;

    RemoteKeyboard keyboard;
};

/**
 * @brief VT02输出数据结构体
 * 
 */
struct RemoteVT02OutputData
{
    RemoteMouse mouse;
    RemoteKeyboard keyboard;
};

/**
 * @brief VT02类
 * 
 */
class RemoteDjiVT02 : public Remote
{
public:
    // 遥控器输出数据
    RemoteVT02OutputData output_;

private:
    // 遥控原始数据
    RmoteVT02RawData raw_data_;

    void ClearData() override;

    void DataProcess(uint8_t* buffer) override;
};


/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/







#endif 