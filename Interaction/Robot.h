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

#include "main.h"

#include "dvc_MCU_comm.h"
#include "dvc_remote_dji.h"

#include "app_gimbal.h"
#include "app_shoot.h"


class Robot
{
public:
    RemoteDjiDR16 remote_dr16_;

    McuComm mcu_comm_;

    Gimbal gimbal_;

    Shoot shoot_;

    void Init();
    void Task();
protected:

    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

#endif