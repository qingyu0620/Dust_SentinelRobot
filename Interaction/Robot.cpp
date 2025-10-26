/**
 * @file app_robot.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "Robot.h"

/* Private macros ------------------------------------------------------------*/

#define K                       1.0f / 66.f
#define C                       512.f / 33.f
#define MAX_ROTATION_SPEED      10.f
#define MAX_RELOAD_SPEED        -10.f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Robot初始化函数
 * 
 */
void Robot::Init()
{
    // 等待云台上电
    HAL_Delay(2000);
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan1, 0x01, 0x00);
    // 云台初始化
    gimbal_.Init();
    // 摩擦轮初始化
    chassis_.Init();

    HAL_Delay(1000);
    static const osThreadAttr_t kRobotTaskAttr = 
    {
        .name = "robot_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Robot任务函数
 * 
 */
void Robot::Task()
{
    McuChassisData mcu_chassis_data_local;
    mcu_chassis_data_local.chassis_speed_x     = 1024;
    mcu_chassis_data_local.chassis_speed_y     = 1024;
    mcu_chassis_data_local.chassis_rotation    = 1024;
    mcu_chassis_data_local.chassis_spin        = CHASSIS_SPIN_DISABLE;

    McuCommData mcu_comm_data_local;
    mcu_comm_data_local.armor = 0;
    mcu_comm_data_local.supercap = 0;
    mcu_comm_data_local.switch_r = Switch_MID;
    mcu_comm_data_local.yaw = 1024;

    for(;;)
    {
        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_chassis_data_local = *const_cast<const McuChassisData*>(&(mcu_comm_.mcu_chassis_data_));
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        __enable_irq();

        chassis_.SetTargetVelocityX(mcu_chassis_data_local.chassis_speed_x * K - C);
        chassis_.SetTargetVelocityY(mcu_chassis_data_local.chassis_speed_y * K - C);
        switch (mcu_chassis_data_local.chassis_spin) 
        {
            case CHASSIS_SPIN_DISABLE:
            {
                chassis_.SetTargetVelocityRotation(mcu_chassis_data_local.chassis_rotation * K - C);
                break;
            }
            case CHASSIS_SPIN_CLOCKWISE:
            {
                chassis_.SetTargetVelocityRotation(MAX_ROTATION_SPEED);
                break;
            }
            case CHASSIS_SPIN_COUNTER_CLOCK_WISE:
            {
                chassis_.SetTargetVelocityRotation(-MAX_ROTATION_SPEED);
                break;
            }
            default:
                chassis_.SetTargetVelocityRotation(mcu_chassis_data_local.chassis_rotation * K - C);
                break;
        }
        switch (mcu_comm_data_local.switch_r)
        {
            case Switch_MID:
            {
                chassis_.SetTargetReloadRotation(0);
                break;
            }
            case Switch_DOWN:
            {
                chassis_.SetTargetReloadRotation(MAX_RELOAD_SPEED);
                break;
            }
            default:
            {
                chassis_.SetTargetReloadRotation(0);
                break;
            }
        }
        osDelay(pdMS_TO_TICKS(10));
    }
}



