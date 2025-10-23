/**
 * @file app_robot.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "Robot.h"

#include "bsp_uart.h"

#include "dvc_MCU_comm.h"

#include "app_gimbal.h"

void Robot::Init()
{
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan1, 0x01, 0x00);
    // 云台初始化
    gimbal_.Init();
    // 摩擦轮初始化
    chassis_.Init();

    HAL_Delay(3000);
    static const osThreadAttr_t kRobotTaskAttr = 
    {
        .name = "robot_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

void Robot::Task()
{
    McuCommData mcu_comm_data_local;
    mcu_comm_data_local.yaw                 = 127;
    mcu_comm_data_local.pitch_angle         = 127;
    mcu_comm_data_local.chassis_speed_x     = 127;
    mcu_comm_data_local.chassis_speed_y     = 127;
    mcu_comm_data_local.chassis_rotation    = 127;
    mcu_comm_data_local.chassis_spin        = CHASSIS_SPIN_DISABLE;
    // mcu_comm_data_local.supercap            = NULL;

    for(;;)
    {
        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        __enable_irq();
    }
}



