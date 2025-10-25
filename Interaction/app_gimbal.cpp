/**
 * @file app_gimbal.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "app_gimbal.h"
#include "can.h"

void Gimbal::Init()
{
    // 4310电机初始化
    motor_pitch_.Init(&hcan2, 0x04, 0x04);

    motor_pitch_.CanSendEnter();
    HAL_Delay(1000);

    motor_pitch_.SetKp(22);     //MIT模式kp

    motor_pitch_.SetKd(0.16);

    motor_pitch_.SetControlOmega(0);

    motor_pitch_.SetControlTorque(0.1);

    motor_pitch_.Output();

    static const osThreadAttr_t kGimbalTaskAttr = {
        .name = "gimbal_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Gimbal::TaskEntry, this, &kGimbalTaskAttr);
}

/**
 * @brief 自身解算
 *
 */
void Gimbal::SelfResolution()
{
    now_pitch_angle_ = motor_pitch_.GetNowAngle();
    // yaw_angle_pid_.SetNow(now_yaw_angle_);
    // yaw_angle_pid_.CalculatePeriodElapsedCallback();
    // target_yaw_omega_ = yaw_angle_pid_.GetOut();

    // // pitch轴角度归化到±PI / 2之间
    // now_pitch_angle_ = Math_Modulus_Normalization(-motor_pitch_.GetNowAngle(), 2.0f * PI);
}

/**
 * @brief 输出到电机
 *
 */
void Gimbal::Output()
{
    // // 云台位控
    // if (gimbal_control_type_ == GIMBAL_CONTROL_TYPE_MANUAL)         // 无自瞄介入
    // {
    //     // do nothing
    // }else if (gimbal_control_type_ == GIMBAL_CONTROL_TYPE_AUTOAIM){ // 有自瞄矫正
    //     MotorNearestTransposition();
    //     yaw_angle_pid_.SetTarget(now_yaw_angle_); // 加视觉的相对偏移量
    //     yaw_angle_pid_.CalculatePeriodElapsedCallback();
    //     target_yaw_omega_ = yaw_angle_pid_.GetOut();
    // }


    motor_pitch_.SetControlAngle(target_pitch_angle_);

    motor_pitch_.Output();
}

/**
 * @brief 电机就近转位
 *
 */
void Gimbal::MotorNearestTransposition()
{
    // Yaw就近转位
    float tmp_delta_angle;
    tmp_delta_angle = fmod(target_yaw_angle_ - now_yaw_angle_, 2.0f * PI);
    if (tmp_delta_angle > PI)
    {
        tmp_delta_angle -= 2.0f * PI;
    }
    else if (tmp_delta_angle < -PI)
    {
        tmp_delta_angle += 2.0f * PI;
    }
    // target_yaw_angle_ = motor_yaw.GetNowAngle() + tmp_delta_angle;

    // // Pitch就近转位
    // Math_Constrain(&target_pitch_angle_, Min_Pitch_Angle, Max_Pitch_Angle);
    // tmp_delta_angle = target_pitch_angle_ - now_pitch_angle_;
    // target_pitch_angle_ = -motor_pitch_.GetNowAngle() + tmp_delta_angle;
}
// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Gimbal::TaskEntry(void *argument)
{
    Gimbal *self = static_cast<Gimbal *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

// 实际任务逻辑（无限循环）
void Gimbal::Task()
{
    for (;;)
    {
        SelfResolution();
        Output();
        osDelay(pdMS_TO_TICKS(10));
    }
}