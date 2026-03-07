/**
 * @file dvc_MCU_comm.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "dvc_PC_comm.h"
#include "dvc_remote_dr16.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct PCRecvAutoAimData;      // 前置声明

/**
 * @brief Mcu转换联合体
 * 
 */
union McuConv
{
    uint8_t b[4];
    float f;
};

/**
 * @brief Mcu存活状态枚举
 * 
 */
enum McuAliveState
{
    MCU_ALIVE_STATE_ENABLE = 0,
    MCU_ALIVE_STATE_DISABLE,
};

/**
 * @brief Mcu底盘数据结构体
 * 
 */
struct McuChassisData
{
    uint8_t          start_of_frame = 0xAA;     // 帧头
    uint16_t         chassis_speed_x;           // 平移方向：左、右
    uint16_t         chassis_speed_y;           // 平移方向：前、后
    uint16_t         rotation;                  // 旋转方向：不转、顺时针转、逆时针转
    union
    {
        uint8_t all;
        struct
        {
            uint8_t switch_l : 2;
            uint8_t switch_r : 2;
            uint8_t reserved : 4;
        };
    } switch_lr;
};

/**
 * @brief Mcu通用数据结构体
 * 
 */
struct McuCommandData
{
    uint8_t         start_of_frame = 0xAB;

    McuConv         imu_yaw;                    // yaw轴角度
};

/**
 * @brief Mcu接收自瞄数据结构体
 * 
 */
struct McuSendAutoData
{
    uint8_t start_of_yaw_frame = 0xAC;
    uint8_t mode;
    McuConv autoaim_yaw_angle;            // 自瞄yaw轴角度
    union 
    {
        uint8_t all;
        struct
        {
            uint8_t chassis_mode : 1;
            uint8_t scan_status : 1;
            uint8_t reserved : 6;
        };
    };
};

/**
 * @brief 
 * 
 */
struct McuRecvRefereeFastData
{
    uint8_t start_of_yaw_frame = 0xAC;

    uint8_t middle_buff_status = 0;
    uint16_t bullet_number = 0;
    McuConv bullet_speed;
};

/**
 * @brief 
 * 
 */
struct McuRecvRefereeSlowData
{
    uint8_t start_of_yaw_frame = 0xAF;

    uint8_t stage_enum = 0;
    uint16_t stage_remain_time = 0;
    uint16_t robot_hp = 0;

    uint8_t is_jam = false;
};

/**
 * @brief Mcu通讯类
 * 
 */
class McuComm
{
public:
    McuChassisData send_chassis_data_ = 
    {
        0xAA,
        1024,
        1024,
        1024,
        15,
    };
    
    McuCommandData send_command_data_ = 
    {
        0xAB,
        {0,0,0,0},
    };

    McuSendAutoData send_auto_data_ = 
    {   0xAC,
        0,
        {0,0,0,0},
        0,
    };

    McuRecvRefereeFastData recv_fast_data_ = 
    {
        0xAC,
        0,
        0,
        {0,0,0,0},
    };

    McuRecvRefereeSlowData recv_slow_data_ = 
    {
        0xAD,
        0,
        0,
        0,
    };

    void Init(CAN_HandleTypeDef *hcan, uint8_t can_rx_id, uint8_t can_tx_id);

    void Task();

    void ClearRemoteData();

    void ClearAutoData();

    void CanSendChassisData();

    void CanSendCommandData();
    
    void CanSendAutoData();

    void CanRxCpltCallback(uint8_t *rx_data);

    inline McuAliveState GetMcuAliveState();

protected:

    CanManageObject *can_manage_object_;

    uint16_t can_rx_id_;

    uint16_t can_tx_id_;

    uint8_t tx_data_[8];

    uint32_t flag_ = 0;

    uint32_t pre_flag_ = 0;

    uint32_t alive_count_ = 0;

    McuAliveState mcu_alive_state_ = MCU_ALIVE_STATE_DISABLE;

    void DataProcess(uint8_t* rx_data);

    void AlivePeriodElapsedCallback();
    
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline McuAliveState McuComm::GetMcuAliveState()
{
    return (mcu_alive_state_);
}






