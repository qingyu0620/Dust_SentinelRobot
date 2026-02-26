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
#include "app_chassis.h"
#include "bsp_uart.h"
#include "dvc_remote_dr16.h"

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
    // supercap_.Init(&hcan2);

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
    mcu_chassis_data_local.chassis_speed_x     = 1024;
    mcu_chassis_data_local.chassis_speed_y     = 1024;
    mcu_chassis_data_local.rotation            = 1024;

    // Mcu命令数据
    McuCommandData mcu_comm_data_local;
    mcu_comm_data_local.keyboard.all           = 0;
    mcu_comm_data_local.mouse_lr.all           = 0;
    mcu_comm_data_local.imu_yaw.f              = 0.0f;

    // Mcu自瞄数据
    McuRecvAutoaimData mcu_autoaim_data_local;
    mcu_autoaim_data_local.mode                = 0;
    mcu_autoaim_data_local.ratio               = 1;
    mcu_autoaim_data_local.autoaim_yaw_ang.f   = 0;

    for(;;)
    {
        /****************************   McuComm   ****************************/


        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_chassis_data_local = *(static_cast<const McuChassisData*>(&(mcu_comm_.recv_chassis_data_)));
        mcu_comm_data_local = *(static_cast<const McuCommandData*>(&(mcu_comm_.recv_comm_data_)));
        mcu_autoaim_data_local = *(static_cast<const McuRecvAutoaimData*>(&(mcu_comm_.recv_autoaim_data_)));
        __enable_irq();


        /****************************   Gimbal   ****************************/


        // 自瞄模式
        if(mcu_chassis_data_local.switch_lr.switchcode.switch_r == SWITCH_UP || mcu_comm_data_local.mouse_lr.mousecode.mouse_r == REMOTE_KEY_STATUS_PRESS)
        {
            switch (mcu_autoaim_data_local.mode) 
            {
                case(PC_AUTOAIM_MODE_IDLE):
                {
                    remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadRotation(0);

                    break;
                }
                case(PC_AUTOAIM_MODE_REAR):
                {
                    remote_yaw_radian_ += (M_PI / 180.f) * REAR_YAW_RATIO;
                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadRotation(0);

                    break;
                }
                case(PC_AUTOAIM_MODE_FRONT):
                {
                    float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_autoaim_data_local.autoaim_yaw_ang.f);

                    remote_yaw_radian_ += (filtered_autoaim / AUTOAIM_YAW_RATIO);
                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);

                    break;
                }
            }
        }
        // 巡航模式
        else if(mcu_chassis_data_local.switch_lr.switchcode.switch_r == SWITCH_DOWN || mcu_comm_data_local.keyboard.keycode.q == REMOTE_KEY_STATUS_PRESS)
        {
            switch (mcu_autoaim_data_local.mode) 
            {
                case(PC_AUTOAIM_MODE_IDLE):
                {
                    remote_yaw_radian_ += (M_PI / 180.f) * SCAN_YAW_RATIO;
                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadRotation(0);

                    break;
                }
                case(PC_AUTOAIM_MODE_REAR):
                {
                    remote_yaw_radian_ += (M_PI / 180.f) * REAR_YAW_RATIO;
                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadRotation(0);

                    break;
                }
                case(PC_AUTOAIM_MODE_FRONT):
                {
                    float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_autoaim_data_local.autoaim_yaw_ang.f);

                    remote_yaw_radian_ += (filtered_autoaim / AUTOAIM_YAW_RATIO);
                    remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

                    gimbal_.SetTargetYawRadian(remote_yaw_radian_);

                    reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);

                    break;
                }
            }
        }
        // 正常模式
        else 
        {
            remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;

            remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

            gimbal_.SetTargetYawRadian(remote_yaw_radian_);
        }


        // 检测MCU掉线
        if(mcu_comm_.GetMcuAliveState() == MCU_ALIVE_STATE_ENABLE)
        {
            gimbal_.SetNowImuYawRadian(normalize_angle_pm_pi(mcu_comm_data_local.imu_yaw.f));
        }
        else if (mcu_comm_.GetMcuAliveState() == MCU_ALIVE_STATE_DISABLE) 
        {
            gimbal_.SetNowImuYawRadian(gimbal_.GetTargetYawRadian());
        }


        /****************************   Chassis   ****************************/


        // 设置当前云台弧度差
        chassis_.SetNowYawRadianDiff(-gimbal_.GetNowYawRadian());

        // 设置目标映射速度
        chassis_.SetTargetVxInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_x + C_NORM) * MAX_OMEGA_SPEED);
        chassis_.SetTargetVyInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_y + C_NORM) * MAX_OMEGA_SPEED);


        /*****************************     Mode   *****************************/

        if(mcu_chassis_data_local.switch_lr.switchcode.switch_l == SWITCH_UP || 
          (mcu_comm_data_local.mouse_lr.mousecode.mouse_r && mcu_comm_data_local.mouse_lr.mousecode.mouse_l) || 
          (mcu_comm_data_local.keyboard.keycode.e && mcu_comm_data_local.mouse_lr.mousecode.mouse_l))
        {
            reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);
        }
        else if(mcu_chassis_data_local.switch_lr.switchcode.switch_l == SWITCH_MID)
        {
            reload_.SetTargetReloadRotation(0);
        }
        
        
        if(mcu_comm_data_local.keyboard.keycode.shift == REMOTE_KEY_STATUS_PRESS)
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_SPIN);
        }
        else if (mcu_comm_data_local.keyboard.keycode.ctrl == REMOTE_KEY_STATUS_PRESS)
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_FOLLOW);
        }
        else
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_NORMAL);
        }


        /****************************   Supercap   ****************************/





        /****************************   Debug   ****************************/

        

        osDelay(pdMS_TO_TICKS(1));
    }
}



