/**
 * @file app_gimbal.h
 * @author qingyu
 * @brief 
 * @version 0.2
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

/* Includes ------------------------------------------------------------------*/

#include "stdio.h"
#include "FreeRTOS.h"
// alg
#include "alg_math.h"
#include "low_pass_filter.hpp"
#include "Timer.hpp"
// module
#include "dvc_motor_dm.h"
// bsp
#include "cmsis_os2.h"
#include "bsp_can.h"

/* Exported macros -----------------------------------------------------------*/

constexpr float K_PITCH           = 13.f / 13200.f;
constexpr float C_PITCH           = -832.f / 825.f;
constexpr float MAX_PITCH_RADIAN  = 0.45f;
constexpr float MIN_PITCH_RADIAN  = -0.35f / 1.5f;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Gimbal控制类型枚举
 *
 */
enum GimbalControlType
{
    GIMBAL_CONTROL_TYPE_MANUAL = 0,
    GIMBAL_CONTROL_TYPE_AUTOAIM,
};

/**
 * @brief Gimbal类
 * 
 */
class Gimbal
{
public:
    // DM4310电机
    MotorDmNormal motor_pitch_;

    // pitch角速度环
    Pid pitch_omega_pid_;

    // pitch角位置环
    Pid pitch_angle_pid_;

    // pitch轴角速度滤波
    LowPassFilter pitch_omega_filter_;

    // pitch轴自瞄数据滤波
    LowPassFilter pitch_autoaim_filter_;

    void Init();

    void Task();

    inline void SetTargetPitchAngle(float target_pitch_angle);

    inline void SetTargetPitchOmega(float target_pitch_omega);

    inline void SetTargetPitchTorque(float target_pitch_torque);

    inline void SetTargetPitchRadian(float target_pitch_radian);

    inline void SetNowImuPitchRadian(float imu_pitch_radian);

    inline float GetNowPitchAngle();

    inline float GetNowPitchOmega();

    inline float GetNowPitchTorque();

    inline float GetNowPitchRadian();

    inline float GetTargetPitchAngle();

    inline float GetTargetPitchOmega();

    inline float GetTargetPitchTorque();

    inline float GetTargetPitchRadian();

protected:
    // 掉线保护定时器
    Timer NoConnectTimer{200};

    // pitch轴当前值
    float now_pitch_angle_ = 0.0f;

    float now_pitch_omega_ = 0.0f;

    float now_pitch_torque_ = 0.0f;

    float now_pitch_radian_ = 0.0f;

    // pitch轴目标值
    float target_pitch_angle_ = 0.0f;

    float target_pitch_omega_ = 0.0f;

    float target_pitch_torque_ = 0.0f;

    float target_pitch_radian_ = 0.0f;

    // pitch轴角度差
    float pitch_angle_diff_ = 0.0f;

    // 陀螺仪pitch轴角度
    float imu_pitch_radian_ = 0.0f;

    // 云台状态
    GimbalControlType gimbal_control_type_ = GIMBAL_CONTROL_TYPE_MANUAL;

    MotorDmControlStatusNormal now_pitch_status_ = MOTOR_DM_CONTROL_STATUS_DISABLE;

    void SelfResolution();

    void Output();

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 获取pitch轴当前角度
 *
 * @return float pitch轴当前角度
 */
inline float Gimbal::GetNowPitchAngle()
{
    return (now_pitch_angle_);
}

/**
 * @brief 获取pitch轴当前角速度
 *
 * @return float pitch轴当前角速度
 */
inline float Gimbal::GetNowPitchOmega()
{
    return (now_pitch_omega_);
}

/**
 * @brief 获取pitch轴当前力矩
 *
 * @return float pitch轴当前力矩
 */
inline float Gimbal::GetNowPitchTorque()
{
    return (now_pitch_torque_);
}

/**
 * @brief 获取pitch轴当前弧度
 *
 * @return float pitch轴当前弧度
 */
inline float Gimbal::GetNowPitchRadian()
{
    return (now_pitch_radian_);
}

/**
 * @brief 获取pitch轴目标角度
 *
 * @return float pitch轴目标角度
 */
inline float Gimbal::GetTargetPitchAngle()
{
    return (target_pitch_angle_);
}

/**
 * @brief 获取pitch轴目标角速度
 *
 * @return float pitch轴目标角速度
 */
inline float Gimbal::GetTargetPitchOmega()
{
    return (target_pitch_omega_);
}

/**
 * @brief 获取pitch轴目标力矩
 *
 * @return float pitch轴目标力矩
 */
inline float Gimbal::GetTargetPitchTorque()
{
    return (target_pitch_torque_);
}

/**
 * @brief 获取pitch轴目标弧度
 *
 * @return float pitch轴目标弧度
 */
inline float Gimbal::GetTargetPitchRadian()
{
    return (target_pitch_radian_);
}

/**
 * @brief 设定pitch轴角度
 *
 * @param target_pitch_angle pitch轴角度
 */
inline void Gimbal::SetTargetPitchAngle(float target_pitch_angle)
{
    target_pitch_angle_ = target_pitch_angle;
}

/**
 * @brief 设定pitch轴角速度
 *
 * @param target_pitch_omega pitch轴角速度
 */
inline void Gimbal::SetTargetPitchOmega(float target_pitch_omega)
{
    target_pitch_omega_ = target_pitch_omega;
}

/**
 * @brief 设定pitch轴力矩
 *
 * @param target_pitch_torque pitch轴力矩
 */
inline void Gimbal::SetTargetPitchTorque(float target_pitch_torque)
{
    target_pitch_torque_ = target_pitch_torque;
}

/**
 * @brief 设定pitch轴力矩
 *
 * @param target_pitch_torque pitch轴力矩
 */
inline void Gimbal::SetTargetPitchRadian(float target_pitch_radian)
{
    target_pitch_radian_ = target_pitch_radian;
}

inline void Gimbal::SetNowImuPitchRadian(float imu_pitch_radian)
{
    imu_pitch_radian_ = imu_pitch_radian;
}


#endif // !GIMBAL_H