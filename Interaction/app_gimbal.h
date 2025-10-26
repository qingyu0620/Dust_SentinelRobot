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

#include "FreeRTOS.h"
// module
#include "dvc_motor_dm.h"
// bsp
#include "cmsis_os2.h"
#include "bsp_can.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台控制类型
 *
 */
enum GimbalControlType
{
    GIMBAL_CONTROL_TYPE_MANUAL = 0,
    GIMBAL_CONTROL_TYPE_AUTOAIM,
};

class Gimbal
{
public:
    // 2个DM6220，作为云台Yaw和Pitch轴控制电机
    MotorDmNormal motor_yaw_;
    MotorDmNormal motor_pitch_;

    Pid yaw_angle_pid_;


    void Init();

    void Task();

    inline float GetNowYawAngle();

    inline float GetNowPitchAngle();

    inline float GetNowYawOmega();

    inline float GetNowPitchOmega();

    inline float GetTargetYawAngle();

    inline float GetTargetPitchAngle();

    inline float GetTargetYawOmega();

    inline float GetTargetPitchOmega();

    inline void SetTargetYawAngle(float target_yaw_angle);

    inline void SetTargetPitchAngle(float target_pitch_angle);

    inline void SetTargetYawOmega(float target_yaw_omega);

    inline void SetTargetPitchOmega(float target_pitch_omega);

protected:
    // pitch轴最小值
    float min_pitch_angle_ = -0.60f;
    // pitch轴最大值
    float max_pitch_angle_ = 0.33f;

    // 内部变量

    // 读变量

    // yaw轴当前角度
    float now_yaw_angle_ = 0.0f;
    // pitch轴当前角度
    float now_pitch_angle_ = 0.0f;

    // yaw轴当前角速度
    float now_yaw_omega_ = 0.0f;
    // pitch轴当前角速度
    float now_pitch_omega_ = 0.0f;

    // 写变量

    // 云台状态
    GimbalControlType gimbal_control_type_ = GIMBAL_CONTROL_TYPE_MANUAL;
    // 读写变量

    // yaw轴目标角度
    float target_yaw_angle_ = 0.0f;
    // pitch轴目标角度
    float target_pitch_angle_ = 0.0f;

    // yaw轴目标角速度
    float target_yaw_omega_ = 0.0f;
    // pitch轴目标角速度
    float target_pitch_omega_ = 0.0f;

    void SelfResolution();
    void MotorNearestTransposition();
    void Output();
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 获取yaw轴当前角度
 *
 * @return float yaw轴当前角度
 */
inline float Gimbal::GetNowYawAngle()
{
    return (now_yaw_angle_);
}

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
 * @brief 获取yaw轴当前角速度
 *
 * @return float yaw轴当前角速度
 */
inline float Gimbal::GetNowYawOmega()
{
    return (now_yaw_omega_);
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
 * @brief 获取yaw轴目标角度
 *
 * @return float yaw轴目标角度
 */
inline float Gimbal::GetTargetYawAngle()
{
    return (target_yaw_angle_);
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
 * @brief 获取yaw轴目标角速度
 *
 * @return float yaw轴目标角速度
 */
inline float Gimbal::GetTargetYawOmega()
{
    return (target_yaw_omega_);
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
 * @brief 设定yaw轴角度
 *
 * @param target_yaw_angle yaw轴角度
 */
inline void Gimbal::SetTargetYawAngle(float target_yaw_angle)
{
    target_yaw_angle_ = target_yaw_angle;
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
 * @brief 设定yaw轴角速度
 *
 * @param target_yaw_omega yaw轴角速度
 */
inline void Gimbal::SetTargetYawOmega(float target_yaw_omega)
{
    target_yaw_omega_ = target_yaw_omega;
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


#endif // !GIMBAL_H