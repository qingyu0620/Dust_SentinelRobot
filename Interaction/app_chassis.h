/**
 * @file app_chassis.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_CHASSIS_H_
#define APP_CHASSIS_H_

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
// device
#include "dvc_motor_dji.h"
// bsp
#include "cmsis_os2.h"
#include "bsp_can.h"
#include "stdio.h"

/* Exported macros -----------------------------------------------------------*/

#define MAX_OMEGA_SPEED         10.f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Chassis控制模式枚举
 * 
 */
enum ChassisOperationMode
{
    CHASSIS_OPERATION_MODE_SPIN = 0,        // 小陀螺模式
    CHASSIS_OPERATION_MODE_NORMAL,          // 正常模式
    CHASSIS_OPERATION_MODE_FOLLOW,          // 底盘跟随
};

/**
 * @brief Chassis类
 * 
 */
class Chassis
{
public:
    // 底盘4个3508，控制全向轮
    MotorDjiC620 motor_chassis_1_,
                 motor_chassis_2_,
                 motor_chassis_3_,
                 motor_chassis_4_;

    // 底盘跟随pid
    Pid chassis_follow_pid_;

    void Init();

    void Task();

    inline void SetChassisOperationMode(ChassisOperationMode chassis_opreation_mode);

    inline void SetTargetVxInGimbal(float target_velocity_x);

    inline void SetTargetVyInGimbal(float target_velocity_y);

    inline void SetTargetVelocityRotation(float target_velocity_rotation);

    inline void SetNowYawRadianDiff(float yaw_radian_diff);

protected:
    // 底盘操作模式
    ChassisOperationMode chassis_opreation_mode_ = CHASSIS_OPERATION_MODE_NORMAL;

    // 云台坐标系目标速度
    float target_vx_in_gimbal_ = 0.0f;
    float target_vy_in_gimbal_ = 0.0f;

    // 底盘坐标系目标速度
    float target_vx_in_chassis_ = 0.0f;
    float target_vy_in_chassis_ = 0.0f;

    // 目标速度 旋转
    float target_velocity_rotation_ = 0.0f;

    // yaw轴角度差
    float yaw_radian_diff_ = 0.0f;

// 斜坡规划参数

    // xyr轴最大加速度

    float max_accel_xy_ = 70.f;
    float max_accel_r_  = 100.f;

    // xyr当前加速度

    float now_accel_x_ = 0.0f;
    float now_accel_y_ = 0.0f;
    float now_accel_r_ = 0.0f;

    // xyr上一次目标速度

    float last_target_vx_ = 0.0f;
    float last_target_vy_ = 0.0f;
    float last_target_rotation_ = 0.0f;

// 底盘驱动

    void OperationMode();

    void RotationMatrixTransform();

    void SlopePlanning();

    void KinematicsInverseResolution();

    void OutputToMotor();

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 切换底盘操作模式
 * 
 * @param chassis_opreation_mode 
 */
inline void Chassis::SetChassisOperationMode(ChassisOperationMode chassis_opreation_mode)
{
    chassis_opreation_mode_ = chassis_opreation_mode;
}

/**
 * @brief 设定目标速度X
 *
 * @param target_velocity_x 目标速度X
 */
inline void Chassis::SetTargetVxInGimbal(float target_vx_in_gimbal)
{
    target_vx_in_gimbal_ = target_vx_in_gimbal;
}

/**
 * @brief 设定目标速度Y
 *
 * @param target_velocity_y 目标速度Y
 */
inline void Chassis::SetTargetVyInGimbal(float target_vy_in_gimbal)
{
    target_vy_in_gimbal_ = target_vy_in_gimbal;
}

/**
 * @brief 设定目标速度旋转
 *
 * @param target_velocity_rotation 目标速度Y
 */
inline void Chassis::SetTargetVelocityRotation(float target_velocity_rotation)
{
    target_velocity_rotation_ = target_velocity_rotation;
}

/**
 * @brief 设定现在yaw角偏差
 * 
 * @param now_yawdiff 
 */
inline void Chassis::SetNowYawRadianDiff(float yaw_radian_diff)
{
    yaw_radian_diff_ = yaw_radian_diff;
}

#endif // !APP_CHASSIS_H_