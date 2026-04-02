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

// 速度模式: 0-保守 1-正常 2-激进
#define CHASSIS_SPEED_MODE 1

#if CHASSIS_SPEED_MODE == 0
constexpr float MAX_GYROSCOPE_SPEED = 12.f;
constexpr float MAX_OMEGA_SPEED = 6.f;

#elif CHASSIS_SPEED_MODE == 1
constexpr float MAX_GYROSCOPE_SPEED = 18.f;
constexpr float MAX_OMEGA_SPEED = 8.7f;

#elif CHASSIS_SPEED_MODE == 2
constexpr float MAX_GYROSCOPE_SPEED = 22.f;
constexpr float MAX_OMEGA_SPEED = 10.f;
#endif

constexpr float VARIABLE_RANG   = 2.5f;
constexpr float VARIABLE_RATIO  = 800.f;

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Chassis初始化函数
 * 
 */
void Chassis::Init()
{
    // 底盘跟随pid
    chassis_follow_pid_.Init(
        15.f,
        0.3f,
        0.005f,
        2.0f,
        0.0f,
        25.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f  
    );
    // 底盘3508电机初始化
    motor_chassis_1_.pid_omega_.Init(2.0f,0.f,0.0007f);
    motor_chassis_2_.pid_omega_.Init(2.0f,0.f,0.0007f);
    motor_chassis_3_.pid_omega_.Init(2.0f,0.f,0.0007f);
    motor_chassis_4_.pid_omega_.Init(2.0f,0.f,0.0007f);

    motor_chassis_1_.Init(&hcan1, MOTOR_DJI_ID_0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA, 3591.0f / 187.0f, MOTOR_DJI_POWER_LIMIT_STATUS_ENABLE);
    motor_chassis_2_.Init(&hcan1, MOTOR_DJI_ID_0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA, 3591.0f / 187.0f, MOTOR_DJI_POWER_LIMIT_STATUS_ENABLE);
    motor_chassis_3_.Init(&hcan1, MOTOR_DJI_ID_0x203, MOTOR_DJI_CONTROL_METHOD_OMEGA, 3591.0f / 187.0f, MOTOR_DJI_POWER_LIMIT_STATUS_ENABLE);
    motor_chassis_4_.Init(&hcan1, MOTOR_DJI_ID_0x204, MOTOR_DJI_CONTROL_METHOD_OMEGA, 3591.0f / 187.0f, MOTOR_DJI_POWER_LIMIT_STATUS_ENABLE);

    motor_chassis_1_.SetTargetOmega(0.0f);
    motor_chassis_2_.SetTargetOmega(0.0f);
    motor_chassis_3_.SetTargetOmega(0.0f);
    motor_chassis_4_.SetTargetOmega(0.0f);

    static const osThreadAttr_t kChassisTaskAttr = 
    {
        .name = "chassis_task",
        .stack_size = 1024,
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
 * @brief Chassis操作模式
 * 
 */
void Chassis::OperationMode()
{
    switch (chassis_opreation_mode_)
    {
        case (CHASSIS_OPERATION_MODE_SPIN):
        {
            variable_spin = 0;

            SetMaxOmegaSpeed(MAX_OMEGA_SPEED / 2.5f);
            SetTargetVelocityRotation(MAX_GYROSCOPE_SPEED);
            break;
        }
        case (CHASSIS_OPERATION_MODE_NORMAL):
        {
            variable_spin = 0;

            SetMaxOmegaSpeed(MAX_OMEGA_SPEED);
            SetTargetVelocityRotation(0);
            break;
        }
        case (CHASSIS_OPERATION_MODE_FOLLOW):
        {
            variable_spin = 0;

            chassis_follow_pid_.SetTarget(0);
            chassis_follow_pid_.SetNow(-yaw_radian_diff_);
            chassis_follow_pid_.CalculatePeriodElapsedCallback();

            SetMaxOmegaSpeed(MAX_OMEGA_SPEED);
            SetTargetVelocityRotation(chassis_follow_pid_.GetOut());
            break;
        }
        case (CHASSIS_OPERATION_MODE_VARIABLE):
        {
            variable_spin += M_PI / VARIABLE_RATIO;
            variable_spin = normalize_pi(variable_spin);

            float spin_speed = MAX_GYROSCOPE_SPEED + VARIABLE_RANG * arm_sin_f32(variable_spin);

            SetMaxOmegaSpeed(MAX_OMEGA_SPEED / 2.f);
            SetTargetVelocityRotation(spin_speed);
            break;
        }
        default:
        {
            SetMaxOmegaSpeed(MAX_OMEGA_SPEED);
            SetTargetVelocityRotation(0);
            break;
        }
        
    }
}

/**
 * @brief Chassis旋转矩阵变换
 * 
 */
void Chassis::RotationMatrixTransform()
{
    float cos_theta_ = cosf(yaw_radian_diff_);
    float sin_theta_ = sinf(yaw_radian_diff_);
    target_vx_in_chassis_ = cos_theta_ * target_vx_in_gimbal_ - sin_theta_ * target_vy_in_gimbal_;
    target_vy_in_chassis_ = sin_theta_ * target_vx_in_gimbal_ + cos_theta_ * target_vy_in_gimbal_;
}

/**
 * @brief 斜坡规划
 * 
 */
void Chassis::SlopePlanning()
{
    constexpr float DT = 0.001f;

    auto RampLimit = [](float &target, float &last, float max_accel, float dt)
    {
        float delta = max_accel * dt;
        if (target > last + delta)
            target = last + delta;
        else if (target < last - delta)
            target = last - delta;
        last = target;
    };

    RampLimit(target_vx_in_chassis_,      last_target_vx_,       max_accel_xy_, DT);
    RampLimit(target_vy_in_chassis_,      last_target_vy_,       max_accel_xy_, DT);
    RampLimit(target_velocity_rotation_,  last_target_rotation_, max_accel_r_,  DT);
}

/**
 * @brief Chassis运动学解析
 * 
 */
void Chassis::KinematicsInverseResolution()
{
    motor_chassis_1_.SetTargetOmega(((-target_vx_in_chassis_ + target_vy_in_chassis_) * M_SQRT2 + target_velocity_rotation_));
    motor_chassis_2_.SetTargetOmega(((-target_vx_in_chassis_ - target_vy_in_chassis_) * M_SQRT2 + target_velocity_rotation_));
    motor_chassis_3_.SetTargetOmega( ((target_vx_in_chassis_ - target_vy_in_chassis_) * M_SQRT2 + target_velocity_rotation_));
    motor_chassis_4_.SetTargetOmega( ((target_vx_in_chassis_ + target_vy_in_chassis_) * M_SQRT2 + target_velocity_rotation_));
}

/**
 * @brief Chassis功率控制
 * 
 */
void Chassis::PowerControl()
{
    MotorDjiC620* motors[4] {
        &motor_chassis_1_, 
        &motor_chassis_2_,
        &motor_chassis_3_, 
        &motor_chassis_4_
    };

    float buffer_sqrt_now = sqrtf((float)referee_buffer_energy_);
    float buffer_sqrt_target = sqrtf(buffer_target_);
    float buffer_error = buffer_sqrt_target - buffer_sqrt_now;
    float buffer_deriv = buffer_error - last_buffer_sqrt_error_;

    last_buffer_sqrt_error_ = buffer_error;

    float pd_out = power_pd_kp_ * buffer_error + power_pd_kd_ * buffer_deriv;
    float max_power = (float)referee_power_limit_ - pd_out;
    if (max_power < 15.0f) max_power = 15.0f;

    float sum_power_estimate = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        float pe = motors[i]->GetPowerEstimate();
        if (pe > 0.0f)
            sum_power_estimate += pe;
    }

    float factor = 1.0f;
    if (sum_power_estimate > max_power && sum_power_estimate > 0.001f)
    {
        factor = max_power / sum_power_estimate;
    }
    if (factor < 0.0f) factor = 0.0f;
    if (factor > 1.0f) factor = 1.0f;

    for (int i = 0; i < 4; i++)
    {
        motors[i]->SetPowerFactor(factor);
    }

    motor_chassis_1_.PowerLimitAfterCalculatePeriodElapsedCallback();
    motor_chassis_2_.PowerLimitAfterCalculatePeriodElapsedCallback();
    motor_chassis_3_.PowerLimitAfterCalculatePeriodElapsedCallback();
    motor_chassis_4_.PowerLimitAfterCalculatePeriodElapsedCallback();
}

/**
 * @brief Chassis电机输出
 * 
 */
void Chassis::OutputToMotor()
{
    motor_chassis_1_.CalculatePeriodElapsedCallback();
    motor_chassis_2_.CalculatePeriodElapsedCallback();
    motor_chassis_3_.CalculatePeriodElapsedCallback();
    motor_chassis_4_.CalculatePeriodElapsedCallback();

    PowerControl();

    // 全向轮底盘电机
    can_send_data(&hcan1, 0x200, g_can1_0x200_tx_data, 8);
}

/**
 * @brief Chassis任务函数
 * 
 */
void Chassis::Task()
{
    for (;;)
    {
        OperationMode();

        RotationMatrixTransform();

        SlopePlanning();

        KinematicsInverseResolution();

        OutputToMotor();

        osDelay(pdMS_TO_TICKS(1));
    }
}

