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
#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "app_gimbal.h"
#include "app_chassis.h"

#include "dvc_MCU_comm.h"


class Robot
{
public:
    // 上下板通信
    McuComm mcu_comm_;
    // 底盘驱动
    Chassis chassis_;
    // yaw角云台
    Gimbal gimbal_;

    void Init();
    void Task();
protected:

    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

#endif