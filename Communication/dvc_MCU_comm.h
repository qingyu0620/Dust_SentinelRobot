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
    Switch_UP    = (uint8_t)1,
    Switch_MID   = (uint8_t)3,
    Switch_DOWN  = (uint8_t)2,
};

/**
 * @brief 底盘旋转模式枚举
 * 
 */
enum ChassisSpinMode
{
    CHASSIS_SPIN_CLOCKWISE          = 1,
    CHASSIS_SPIN_DISABLE            = 3,
    CHASSIS_SPIN_COUNTER_CLOCK_WISE = 2,
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
    uint8_t          switch_l;                  // 小陀螺：不转、顺时针转、逆时针转
};

/**
 * @brief Mcu通用数据结构体
 * 
 */
struct McuCommData
{
    uint8_t         start_of_frame = 0xAB;
    uint8_t         armor;                      // 自瞄
    uint8_t         supercap;                   // 超级电容：充电、放电
    uint8_t         switch_r;
    McuConv         imu_yaw;                  // yaw轴角度
};

/**
 * @brief Mcu自瞄数据结构体
 * 
 */
struct McuAutoaimData
{
    uint8_t         start_of_yaw_frame = 0xAC;
    uint8_t         start_of_pitch_frame = 0xAD;
    McuConv         autoaim_yaw;                // 自瞄yaw角
    McuConv         autoaim_pitch;              // 自瞄pitch角
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
            CHASSIS_SPIN_DISABLE,
    };
    McuCommData recv_comm_data_ = 
    {
            0xAB,
            0,
            0,
            Switch_MID,
            {0, 0, 0, 0},
    };

    McuAutoaimData recv_autoaim_data_ = 
    {   0xAC,
        0xAD,
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    };

    McuAutoaimData send_autoaim_data_ = 
    {   0xAC,
        0xAD,
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    };

    void Init(CAN_HandleTypeDef *hcan,
              uint8_t can_rx_id,
              uint8_t can_tx_id);

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
