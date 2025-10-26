/**
 * @file app_shoot.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_SHOOT_H_
#define APP_SHOOT_H_

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"

#include "dvc_motor_dji.h"

#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Shoot
{
public:
    // 摩擦轮2个
    MotorDjiC620 motor_shoot_1_,
                 motor_shoot_2_;

    void Init();
    void Task();
    inline void SetTargetShootSpeed(float target_shoot_speed);

protected:
    // 目标发射速度
    float target_shoot_speed_ = 0.0f;

    static void TaskEntry(void *param);   // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 设定目标发射速度
 * 
 * @param target_shoot_speed 
 */
inline void Shoot::SetTargetShootSpeed(float target_shoot_speed)
{
    target_shoot_speed_ = target_shoot_speed;
}





#endif