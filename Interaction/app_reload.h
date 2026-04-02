/**
 * @file app_reload.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __APP_RELOAD_H__
#define __APP_RELOAD_H__

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "stdio.h"

/* Exported macros -----------------------------------------------------------*/

constexpr float MAX_RELOAD_SPEED = -8.5f;

/* Exported types ------------------------------------------------------------*/

class Reload
{
public:
    // 拨弹盘1个2006，控制进退弹
    MotorDjiC610 motor_reload_1_;

    void Init();
    
    void Task();

    inline void SetTargetReloadRotation(float target_reload_rotation);

    inline bool GetReloadState() {
        return is_jaming;
    }

    inline void SetReloadHeat(uint16_t Shooting_heat) {
        shooting_heat_ = Shooting_heat;
    }

protected:

    uint16_t shooting_heat_ = 0;

    bool is_jaming = false;

    bool is_overheat = false;

    // 目标装载速度 旋转
    float target_reload_rotation_ = 0.0f;

    float reload_now_angle = 0.0f;

    float reload_pre_angld = 0.0f;

    void OutputToMotor();

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 设定目标装载速度旋转
 * 
 * @param target_reload_rotation 
 */
inline void Reload::SetTargetReloadRotation(float target_reload_rotation)
{
    target_reload_rotation_ = target_reload_rotation;
}


#endif