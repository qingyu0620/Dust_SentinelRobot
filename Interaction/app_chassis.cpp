/**
 * @file app_chassis.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "app_chassis.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Chassis初始化函数
 * 
 */
void Chassis::Init()
{
    // 2006电机初始化（拨弹盘电机）
    motor_reload_1_.pid_omega_.Init(1.0f, 0.0f, 0.0f);

    motor_reload_1_.Init(&hcan1, MOTOR_DJI_ID_0x205, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    
    motor_reload_1_.SetTargetOmega(0.0f);

    // 3508电机初始化（底盘电机）
    motor_chassis_1_.pid_omega_.Init(3.f,0.2f,0.0f);
    motor_chassis_2_.pid_omega_.Init(3.f,0.2f,0.0f);
    motor_chassis_3_.pid_omega_.Init(3.f,0.2f,0.0f);
    motor_chassis_4_.pid_omega_.Init(3.f,0.2f,0.0f);

    motor_chassis_1_.Init(&hcan1, MOTOR_DJI_ID_0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_2_.Init(&hcan1, MOTOR_DJI_ID_0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_3_.Init(&hcan1, MOTOR_DJI_ID_0x203, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_4_.Init(&hcan1, MOTOR_DJI_ID_0x204, MOTOR_DJI_CONTROL_METHOD_OMEGA);

    motor_chassis_1_.SetTargetOmega(0.0f);
    motor_chassis_2_.SetTargetOmega(0.0f);
    motor_chassis_3_.SetTargetOmega(0.0f);
    motor_chassis_4_.SetTargetOmega(0.0f);

    static const osThreadAttr_t kChassisTaskAttr = 
    {
        .name = "chassis_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Chassis::TaskEntry, this, &kChassisTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Chassis::TaskEntry(void *argument)
{
    Chassis *self = static_cast<Chassis *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Chassis任务函数
 * 
 */
void Chassis::Task()
{
    for (;;)
    {
        // 设置拨弹速度
        motor_reload_1_ .SetTargetOmega( target_reload_rotation_);

        motor_reload_1_ .CalculatePeriodElapsedCallback();

        // 设置平移速度 + 自旋速度（发送转速rad / s）
        motor_chassis_1_.SetTargetOmega(-target_velocity_x_ + target_velocity_rotation_);
        motor_chassis_2_.SetTargetOmega( target_velocity_y_ + target_velocity_rotation_);
        motor_chassis_3_.SetTargetOmega( target_velocity_x_ + target_velocity_rotation_);
        motor_chassis_4_.SetTargetOmega(-target_velocity_y_ + target_velocity_rotation_);
        
        motor_chassis_1_.CalculatePeriodElapsedCallback();
        motor_chassis_2_.CalculatePeriodElapsedCallback();
        motor_chassis_3_.CalculatePeriodElapsedCallback();
        motor_chassis_4_.CalculatePeriodElapsedCallback();
        
        // 全向轮底盘电机
        can_send_data(&hcan1, 0x200, g_can1_0x200_tx_data, 8);
        can_send_data(&hcan1, 0x1FF, g_can1_0x1ff_tx_data, 2);
        osDelay(pdMS_TO_TICKS(10));
    }
}

