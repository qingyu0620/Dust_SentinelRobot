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
#include "alg_math.h"
#include "app_chassis.h"
#include "app_reload.h"
#include "bsp_uart.h"
#include "dvc_remote_dr16.h"
#include "Timer.hpp"

/* Private macros ------------------------------------------------------------*/

#define K_NORM                  1.f / 660.f
#define C_NORM                  -256.f / 165.f

#define REMOTE_YAW_RATIO        0.5f
#define AUTOAIM_YAW_RATIO       120.f
#define SCAN_YAW_RATIO          0.16f
#define REAR_YAW_RATIO          0.22f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Robot初始化函数
 * 
 */
void Robot::Init()
{
    dwt_init(168);

    // 裁判系统初始化
    referee_.Init(&huart6, uart6_callback_function, UART_BUFFER_LENGTH);

    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan2, 0x01, 0x00);

    // 云台初始化
    gimbal_.Init();

    // 底盘初始化
    chassis_.Init();

    // 拨弹盘初始化
    reload_.Init();

    // 超级电容初始化 
    supercap_.Init(&hcan2);

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
    // Mcu底盘数据
    McuChassisData mcu_chassis_data_local;
    mcu_chassis_data_local.chassis_speed_x      = 1024;
    mcu_chassis_data_local.chassis_speed_y      = 1024;
    mcu_chassis_data_local.rotation             = 1024;

    // Mcu命令数据
    McuCommandData mcu_comm_data_local;
    mcu_comm_data_local.imu_yaw                 = 0.0f;
    mcu_comm_data_local.first_power_on          = true;

    // Mcu自瞄数据
    McuRecvAutoData mcu_auto_data_local;
    mcu_auto_data_local.mode                    = 0;
    mcu_auto_data_local.autoaim_yaw_ang.f       = 0;
    mcu_auto_data_local.all                     = 0;
    
    uint32_t print_count = 0;

    for(;;)
    {
        /****************************   McuComm   ****************************/


        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_chassis_data_local = *(static_cast<const McuChassisData*>(&(mcu_comm_.recv_chassis_data_)));
        mcu_comm_data_local = *(static_cast<const McuCommandData*>(&(mcu_comm_.recv_comm_data_)));
        mcu_auto_data_local = *(static_cast<const McuRecvAutoData*>(&(mcu_comm_.recv_autoaim_data_)));
        __enable_irq();

        if (mcu_comm_data_local.first_power_on) {
            remote_yaw_radian_ = 0;
            mcu_comm_data_local.first_power_on = false;
        } 

        // 检测MCU掉线
        if(mcu_comm_.GetMcuAliveState() == MCU_ALIVE_STATE_ENABLE)
        {
            if (mcu_comm_.first_power_on) 
            {
                remote_yaw_radian_ = normalize_angle_pm_pi(mcu_comm_data_local.imu_yaw);
                mcu_comm_.first_power_on = false;
            }

            gimbal_.SetNowImuYawRadian(normalize_angle_pm_pi(mcu_comm_data_local.imu_yaw));
        }
        else if (mcu_comm_.GetMcuAliveState() == MCU_ALIVE_STATE_DISABLE) 
        {
            if (mcu_comm_.first_power_on) 
            {
                remote_yaw_radian_ = gimbal_.GetTargetYawRadian();
                mcu_comm_.first_power_on = false;
            }

            gimbal_.SetNowImuYawRadian(gimbal_.GetTargetYawRadian());
        }


        /****************************   Gimbal   ****************************/


        // 自瞄模式
        if (mcu_chassis_data_local.switch_lr.switch_r == SWITCH_UP)
        {
            switch (mcu_auto_data_local.mode)
            {
                case(PC_AUTOAIM_MODE_IDLE):
                {
                    remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;
                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    break;
                }
                case(PC_AUTOAIM_MODE_REAR):
                {
                    remote_yaw_radian_ += (M_PI / 180.f) * REAR_YAW_RATIO;
                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    break;
                }
                case(PC_AUTOAIM_MODE_FRONT):
                {
                    float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_auto_data_local.autoaim_yaw_ang.f);

                    remote_yaw_radian_ += (filtered_autoaim / AUTOAIM_YAW_RATIO);
                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    break;
                }
            }
        }
        // 遥控模式
        else if (mcu_chassis_data_local.switch_lr.switch_r == SWITCH_MID)
        {
            remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;
            remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

            gimbal_.SetTargetYawRadian(remote_yaw_radian_);

            reload_.SetTargetReloadRotation(0);
        }
        // 巡航模式
        else if (mcu_chassis_data_local.switch_lr.switch_r == SWITCH_DOWN)
        {
            if (mcu_auto_data_local.scan_status == true)
            {
                switch (mcu_auto_data_local.mode)
                {
                    case(PC_AUTOAIM_MODE_IDLE):
                    {
                        remote_yaw_radian_ += (M_PI / 180.f) * SCAN_YAW_RATIO;
                        remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                        gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                        break;
                    }
                    case(PC_AUTOAIM_MODE_REAR):
                    {
                        remote_yaw_radian_ += (M_PI / 180.f) * REAR_YAW_RATIO;
                        remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                        gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                        break;
                    }
                    case(PC_AUTOAIM_MODE_FRONT):
                    {
                        float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_auto_data_local.autoaim_yaw_ang.f);

                        remote_yaw_radian_ += (filtered_autoaim / AUTOAIM_YAW_RATIO);
                        remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                        gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                        break;
                    }
                }
            }
            else 
            {
                remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;
                remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                reload_.SetTargetReloadRotation(0);
            }
        }


        /****************************   Chassis   ****************************/


        chassis_.SetNowYawRadianDiff(-gimbal_.GetNowYawRadian());

        // 设置目标映射速度
        chassis_.SetTargetVxInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_x + C_NORM) * MAX_OMEGA_SPEED);
        chassis_.SetTargetVyInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_y + C_NORM) * MAX_OMEGA_SPEED);


        /*****************************     Mode   *****************************/


        if (mcu_chassis_data_local.switch_lr.switch_l == SWITCH_UP) {
            reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);
        } else if (mcu_chassis_data_local.switch_lr.switch_l == SWITCH_MID) {
            reload_.SetTargetReloadRotation(0);
        } else if (mcu_auto_data_local.fire) {
            reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);
        }

        if (mcu_auto_data_local.chassis_mode || mcu_chassis_data_local.switch_lr.switch_l == SWITCH_DOWN) {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_SPIN);
        } else {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_NORMAL);
        }


        /****************************   Supercap   ****************************/





        /****************************   Debug   ****************************/

        if (print_count++ >= 20)
        {
            // printf("%d,%f\n", mcu_auto_data_local.fire, remote_yaw_radian_);
            // printf("%f\n", remote_yaw_radian_);
            print_count = 0;
        }
        
        osDelay(pdMS_TO_TICKS(1));
    }
}



