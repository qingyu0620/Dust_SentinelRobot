/**
 * @file app_robot.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bsp_dwt.h"
#include "imu.hpp"

#include "dvc_remote_dr16.h"
#include "dvc_MCU_comm.h"
#include "dvc_PC_comm.h"
// #include "referee.h"

#include "app_gimbal.h"
#include "app_shoot.h"

#include "stdio.h"

/* Exported macros -----------------------------------------------------------*/

/**
 * @brief Robot导航开启状态
 * 
 */
enum RobotControlMethod
{
    ROBOT_CONTROL_METHOD_REMOTE = 0,
    ROBOT_CONTROL_METHOD_NAVIGATION,
};

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Robot类
 * 
 */
class Robot
{
public:
    // 遥控
    RemoteDjiDR16 remote_dr16_;

    // 上下板通讯
    McuComm mcu_comm_;

    // pitch角云台
    Gimbal gimbal_;

    // 摩擦轮
    Shoot shoot_;

    // 上位机通讯
    PcComm pc_comm_;

    // 云台陀螺仪
    Imu imu_;

    void Init();

    void Task();

    RobotControlMethod robot_control_method_ = ROBOT_CONTROL_METHOD_REMOTE;

    void SetControlMethod(RobotControlMethod robot_control_method);

protected:

    float remote_radian = 0.0f;

    float scan_pitch_count_ = 0.0f;

    // 机器人等级
    int32_t robot_level_ = 10;

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

