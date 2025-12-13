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
#ifndef MODULES_COMM_DVC_MCU_COMM_H
#define MODULES_COMM_DVC_MCU_COMM_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "string.h"
#include "stdio.h"

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
 * @brief 遥控状态枚举
 * 
 */
enum RemoteSwitchStatus
{
    SWITCH_UP    = (uint8_t)1,
    SWITCH_MID   = (uint8_t)3,
    SWITCH_DOWN  = (uint8_t)2,
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
    uint8_t         supercap;                   // 超级电容：充电、放电
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
};

/**
 * @brief Mcu发送自瞄数据结构体
 * 
 */
struct McuSendAutoaimData
{
    uint8_t         start_of_yaw_frame = 0xAC;

    McuConv         autoaim_yaw_ang;            // 自瞄累加弧度
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
    };

    McuSendAutoaimData send_autoaim_data_ = 
    {   0xAC,
        {0, 0, 0, 0},
    };

    void Init(CAN_HandleTypeDef *hcan, uint8_t can_rx_id, uint8_t can_tx_id);

    void CanRxCpltCallback(uint8_t *rx_data);

    void CanSendCommand();

    void CanSendAutoaim();

    void Task();


protected:

    CanManageObject *can_manage_object_;

    uint16_t can_rx_id_;

    uint16_t can_tx_id_;

    uint8_t tx_data_[8];

    void DataProcess();

    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

#endif
