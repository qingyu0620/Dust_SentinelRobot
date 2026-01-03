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
#ifndef DVC_MCU_COMM_H
#define DVC_MCU_COMM_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "string.h"
#include "stdio.h"
#include "dvc_remote_dji.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 转换联合体
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
    uint16_t         chassis_speed_x;           // 平移方向
    uint16_t         chassis_speed_y;           // 平移方向
    uint16_t         rotation;                  // 旋转方向
};

/**
 * @brief Mcu通用数据结构体
 * 
 */
struct McuCommData
{
    uint8_t         start_of_frame = 0xAB;
    uint8_t         switch_l;                   // 遥控左按钮
    uint8_t         switch_r;                   // 遥控右按钮
    uint8_t         supercap;                   // 超级电容 0-充电 1-放电
    McuConv         imu_yaw;                    // yaw轴角度
};

/**
 * @brief Mcu接收自瞄数据结构体
 * 
 */
struct McuRecvAutoaimData
{
    uint8_t         start_of_yaw_frame = 0xAC;
    uint8_t         mode;                       // 0-空闲 1-自瞄不开火 2-自瞄开火
    McuConv         autoaim_yaw_ang;            // 自瞄yaw轴角度
    uint8_t         flag;                       // 前后摄像头标志位 1-前置 2-后置
};

/**
 * @brief Mcu通讯类
 * 
 */
class McuComm
{
public:
    McuChassisData recv_chassis_data_ = 
    {
        0xAA,
        1024,
        1024,
        1024,
    };

    McuCommData recv_comm_data_ = 
    {
        0xAB,
        SWITCH_MID,
        SWITCH_MID,
        0,
        {0, 0, 0, 0},
    };

    McuRecvAutoaimData recv_autoaim_data_ = 
    {   0xAC,
        0,
        {0, 0, 0, 0},
        0,
    };

    void Init(CAN_HandleTypeDef *hcan, uint8_t can_rx_id, uint8_t can_tx_id);

    void Task();

    void ClearData();

    void CanRxCpltCallback(uint8_t *rx_data);

    inline McuAliveState GetMcuAliveState();

private:

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

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

inline McuAliveState McuComm::GetMcuAliveState()
{
    return (mcu_alive_state_);
}

#endif
