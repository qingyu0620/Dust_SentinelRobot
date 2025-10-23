/**
 * @file app_chassis.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_CHASSIS_H_
#define APP_CHASSIS_H_

#include "FreeRTOS.h"
// device
#include "dvc_motor_dji.h"
#include "dvc_remote_dji.h"
// bsp
#include "cmsis_os2.h"
#include "bsp_can.h"


class Chassis
{
public:
    // 底盘4个3508，控制全向轮
    MotorDjiC620 motor_chassis_1_,
                 motor_chassis_2_,
                 motor_chassis_3_,
                 motor_chassis_4_;
    // 拨弹盘1个2006，控制进退弹
    MotorDjiC610 motor_reload_1_;

    void Init();
    void Task();
    inline void SetTargetVelocityX(float target_velocity_x);
    inline void SetTargetVelocityY(float target_velocity_y);
    inline void SetTargetVelocityRotation(float target_velocity_rotation);
    inline void SetTargetReloadRotation(float target_reload_rotation);
protected:
    // 目标速度X
    float target_velocity_x_ = 0.0f;
    // 目标速度Y
    float target_velocity_y_ = 0.0f;
    // 目标速度 旋转
    float target_velocity_rotation_ = 0.0f;

    // 目标装载速度 旋转
    float target_reload_rotation_ = 0.0f;

    void KinematicsInverseResolution();
    void OutputToMotor();
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/**
 * @brief 设定目标速度X
 *
 * @param target_velocity_x 目标速度X
 */
inline void Chassis::SetTargetVelocityX(float target_velocity_x)
{
    target_velocity_x_ = target_velocity_x;
}

/**
 * @brief 设定目标速度Y
 *
 * @param target_velocity_y 目标速度Y
 */
inline void Chassis::SetTargetVelocityY(float target_velocity_y)
{
    target_velocity_y_ = target_velocity_y;
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
 * @brief 设定目标装载速度旋转
 * 
 * @param target_reload_rotation 
 */
inline void Chassis::SetTargetReloadRotation(float target_reload_rotation)
{
    target_reload_rotation_ = target_reload_rotation;
}

#endif // !APP_CHASSIS_H_