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

#include "cmsis_os2.h"
#include "dvc_MCU_comm.h"
#include "dvc_remote_dji.h"

#include "app_gimbal.h"
#include "app_shoot.h"


#define MAX_SHOOT_SPEED     10.f


void Robot::Init()
{
    // 遥控初始化
    remote_dr16_.Init(&huart3, uart3_callback_function, UART_BUFFER_LENGTH);
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan1, 0x00, 0x01);
    // 云台初始化
    gimbal_.Init();
    // 摩擦轮初始化
    shoot_.Init();

    HAL_Delay(1000);
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
    for(;;)
    {
        // 将遥控器数据发给下板
        mcu_comm_.CanSendCommand();
        // 设置pitch角
        gimbal_.SetTargetPitchAngle(remote_dr16_.output.gimbal_pitch);
        // 摩擦轮转速
        switch (mcu_comm_.mcu_comm_data_.switch_r)
        {
            case Switch_UP:
            {
                shoot_.SetTargetShootSpeed(MAX_SHOOT_SPEED);
                break;
            }
            case Switch_MID:
            {
                shoot_.SetTargetShootSpeed(0);
                break;
            }
            default:
            {
                shoot_.SetTargetShootSpeed(0);
                break;
            }
        }
        
        osDelay(pdMS_TO_TICKS(10));
    }
}



