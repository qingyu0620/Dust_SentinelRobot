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

#include "app_gimbal.h"
#include "alg_math.h"
#include "ins_task.h"

/* Private macros ------------------------------------------------------------*/

#define K_MOTOR_ANGLE      14.4f
#define C_MOTOR_ANGLE      42.f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Gimbal初始化函数
 * 
 */
void Gimbal::Init()
{
    // pitch轴角度环
    pitch_angle_pid_.Init(
        13.8f,
        0.0f,
        0.153f,
        0.0f,
        0.f,
        44.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );
    // pitch轴角度环
    pitch_omega_pid_.Init(
        0.7f,
        0.003f,
        0.00005f,
        0.0f,
        0.0f,
        9.9f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );
    // pitch轴角度环滤波器
    pitch_omega_filter_.Init(15.f, 0.001f);

    pitch_autoaim_filter_.Init(20.f, 0.001f);

    // 4310电机初始化
    motor_pitch_.Init(&hcan2, 0x05, 0x04);

    motor_pitch_.CanSendClearError();
    osDelay(pdMS_TO_TICKS(1000));

    // motor_pitch_.CanSendSaveZero();
    // osDelay(pdMS_TO_TICKS(1000));

    motor_pitch_.CanSendEnter();
    osDelay(pdMS_TO_TICKS(1000));

    motor_pitch_.SetKp(0);      //MIT模式kp
    motor_pitch_.SetKd(0);
    motor_pitch_.SetControlTorque(0);
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
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Gimbal::TaskEntry(void *argument)
{
    Gimbal *self = static_cast<Gimbal *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Gimbal自身解算函数
 *
 */
void Gimbal::SelfResolution()
{
    // 获取当前数据
    now_pitch_omega_ = motor_pitch_.GetNowOmega();                                          // 角速度
    now_pitch_angle_ = K_MOTOR_ANGLE * motor_pitch_.GetNowAngle() + C_MOTOR_ANGLE;          // 角度
    now_pitch_radian_ = normalize_angle_pm_pi(now_pitch_angle_);                     // 弧度
        
    // 计算pitch轴偏差
    pitch_angle_diff_ = imu_pitch_angle_ - target_pitch_radian_;

    // 角度环
    pitch_angle_pid_.SetTarget(0);
    pitch_angle_pid_.SetNow(pitch_angle_diff_);
    pitch_angle_pid_.CalculatePeriodElapsedCallback();

    // 速度环
    pitch_omega_pid_.SetTarget(pitch_angle_pid_.GetOut());
    float filtered_omega = pitch_omega_filter_.Update(now_pitch_omega_);
    pitch_omega_pid_.SetNow(filtered_omega);
    pitch_omega_pid_.CalculatePeriodElapsedCallback();

    // 设定目标力矩
    SetTargetPitchTorque(pitch_omega_pid_.GetOut());
}

/**
 * @brief Gimbal输出函数
 *
 */
void Gimbal::Output()
{
    motor_pitch_.SetControlTorque(target_pitch_torque_);
    motor_pitch_.Output();
}

/**
 * @brief Gimbal就近转位函数
 *
 */
void Gimbal::MotorNearestTransposition()
{
    
}

/**
 * @brief Gimbal任务函数
 * 
 */
void Gimbal::Task()
{
    for (;;)
    {
        SelfResolution();
        Output();
        osDelay(pdMS_TO_TICKS(1));
    }
}