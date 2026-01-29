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

#include "dvc_remote.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief DjiDR16按键状态
 * 
 */
enum RemoteDR16SwitchStatus
{
    SWITCH_UP    = 1,
    SWITCH_MID   = 3,
    SWITCH_DOWN  = 2,
};

/**
 * @brief DjiDR16原始数据
 * 
 */
struct RemoteDR16RawData
{
    struct 
    {
        uint16_t ch0, ch1, ch2, ch3;
        uint8_t s1, s2;
    } rc;

    struct
    {
        int32_t x, y, z;
        uint8_t pl, pr;
    } mouse;

    RemoteKeyboard keyboard;
};

/**
 * @brief DjiDR16输出
 * 
 */
struct RemoteDR16OutputData
{
    struct 
    {
        uint8_t switch_l, switch_r;
        float chassis_x, chassis_y;      // x, y, r 采用右手系
        float rotation;
        float pitch;
    } remote;                            // 遥控数据

    RemoteMouse mouse;

    RemoteKeyboard keyboard;         // 键盘数据
};

/**
 * @brief DjiDR16遥控器
 * 
 */
class RemoteDjiDR16 : public Remote
{
public:
    // 遥控器输出数据
    RemoteDR16OutputData output_;

protected:
    // 原始数据
    RemoteDR16RawData raw_data_;

    void ClearData() override;

    void DataProcess(uint8_t* buffer) override;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif