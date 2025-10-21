/**
 * @file app_shoot.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "app_shoot.h"


void Shoot::Init()
{
    motor_shoot_1_.pid_omega_.Init(1.0f, 0.0f, 0.0f);
    motor_shoot_2_.pid_omega_.Init(1.0f, 0.0f, 0.0f);

    motor_shoot_1_.Init(&hcan2, MOTOR_DJI_ID_0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_shoot_2_.Init(&hcan2, MOTOR_DJI_ID_0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA);

    motor_shoot_1_.SetTargetOmega(0.0f);
    motor_shoot_2_.SetTargetOmega(0.0f);

    static const osThreadAttr_t kShootTaskAttr = 
    {
        .name = "shoot_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Shoot::TaskEntry, this, &kShootTaskAttr);
}

void Shoot::TaskEntry(void *argument)
{
    Shoot *self = static_cast<Shoot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

void Shoot::Task()
{
    for(;;)
    {
        // 摩擦轮对称旋转
        motor_shoot_1_.SetTargetOmega(-target_shoot_speed_);
        motor_shoot_2_.SetTargetOmega( target_shoot_speed_);
        
        motor_shoot_1_.CalculatePeriodElapsedCallback();
        motor_shoot_2_.CalculatePeriodElapsedCallback();
        
        can_send_data(&hcan2, 0x200, g_can2_0x200_tx_data, 4);
        osDelay(pdMS_TO_TICKS(10));
    }
}


