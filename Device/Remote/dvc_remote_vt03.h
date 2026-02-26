/**
 * @file dvc_remote_vt03.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-02-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __DVC_REMOTE_DJI_VT03_H__
#define __DVC_REMOTE_DJI_VT03_H__

/* Includes ------------------------------------------------------------------*/

#include "dvc_remote.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief VT03类
 * 
 */
class RemoteDjiVT03 : public Remote
{
public:

    struct OutputData
    {
        RemoteMouse mouse;
        RemoteKeyboard keyboard;
    };

    // 遥控器输出数据
    OutputData output_;

private:

    struct RawData
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

    // 遥控原始数据
    RawData raw_data_;

    void ClearData() override;

    void DataProcess(uint8_t* buffer) override;
};


/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/







#endif 