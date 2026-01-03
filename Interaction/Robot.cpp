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
#include "dvc_MCU_comm.h"
#include "supercap.h"

/* Private macros ------------------------------------------------------------*/

#define K                       1.f / 660.f
#define C                       -256.f / 165.f
#define MAX_OMEGA_SPEED         15.f
#define MAX_GYROSCOPE_SPEED     25.f

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

    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan2, 0x01, 0x00);

    // 云台初始化
    gimbal_.Init();

    // 底盘陀螺仪初始化
    imu_.Init();

    // 10s时间等待陀螺仪收敛
    osDelay(pdMS_TO_TICKS(10 * 1000));

    // 摩擦轮初始化
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
    mcu_chassis_data_local.chassis_speed_x     = 1024;
    mcu_chassis_data_local.chassis_speed_y     = 1024;
    mcu_chassis_data_local.rotation            = 1024;

    // Mcu命令数据
    McuCommData mcu_comm_data_local;
    mcu_comm_data_local.switch_l               = SWITCH_MID;
    mcu_comm_data_local.switch_r               = SWITCH_MID;
    mcu_comm_data_local.supercap               = 0;
    mcu_comm_data_local.imu_yaw.f              = 0;

    // Mcu自瞄数据
    McuRecvAutoaimData mcu_autoaim_data_local;
    mcu_autoaim_data_local.mode                = 0;
    mcu_autoaim_data_local.flag                = 0;
    mcu_autoaim_data_local.autoaim_yaw_ang.f   = 0;

    for(;;)
    {
        /****************************   McuComm   ****************************/


        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_chassis_data_local = *(static_cast<const McuChassisData*>(&(mcu_comm_.recv_chassis_data_)));
        mcu_comm_data_local = *(static_cast<const McuCommData*>(&(mcu_comm_.recv_comm_data_)));
        mcu_autoaim_data_local = *(static_cast<const McuRecvAutoaimData*>(&(mcu_comm_.recv_autoaim_data_)));
        __enable_irq();


        /****************************   Gimbal   ****************************/

        if(mcu_comm_data_local.switch_r == SWITCH_MID)
        {
            remote_yaw_angle_ += (M_PI / 180.f * (K * mcu_chassis_data_local.rotation + C)) * 0.5;

            remote_yaw_angle_ = normalize_pi(remote_yaw_angle_);

            gimbal_.SetTargetYawRadian(remote_yaw_angle_);
            
        }
        else if(mcu_comm_data_local.switch_r == SWITCH_UP)
        {
            switch (mcu_autoaim_data_local.mode) 
            {
                case(AUTOAIM_MODE_IDIE):
                {
                    remote_yaw_angle_ += (M_PI / 180.f * (K * mcu_chassis_data_local.rotation + C)) * 0.005;

                    remote_yaw_angle_ = normalize_pi(remote_yaw_angle_);

                    gimbal_.SetTargetYawRadian(remote_yaw_angle_);

                    break;
                }
                case(AUTOAIM_MODE_FOLLOW):
                {
                    float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_autoaim_data_local.autoaim_yaw_ang.f);

                    if(mcu_autoaim_data_local.flag == 1){
                        remote_yaw_angle_ += (filtered_autoaim / 120.f);
                    } else if(mcu_autoaim_data_local.flag == 2){
                        remote_yaw_angle_ -= (filtered_autoaim / 480.f);
                    }
                        
                    gimbal_.SetTargetYawRadian(remote_yaw_angle_);

                    break;
                }
                case(AUTOAIM_MODE_FIRE):
                {
                    float filtered_autoaim = gimbal_.yaw_autoaim_filter_.Update(mcu_autoaim_data_local.autoaim_yaw_ang.f);

                    if(mcu_autoaim_data_local.flag == 1){
                        remote_yaw_angle_ += (filtered_autoaim / 120.f);
                    } else if(mcu_autoaim_data_local.flag == 2){
                        remote_yaw_angle_ -= (filtered_autoaim / 480.f);
                    }

                    gimbal_.SetTargetYawRadian(remote_yaw_angle_);
                    // reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);

                    break;
                }
            }
        }


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
        chassis_.SetTargetVxInGimbal((mcu_chassis_data_local.chassis_speed_x * K + C) * MAX_OMEGA_SPEED);
        chassis_.SetTargetVyInGimbal((mcu_chassis_data_local.chassis_speed_y * K + C) * MAX_OMEGA_SPEED);


        /*****************************     Mode   *****************************/

        
        // 左按钮
        switch (mcu_comm_data_local.switch_l) 
        {
            case SWITCH_UP:
            {
                chassis_.SetTargetVelocityRotation(MAX_GYROSCOPE_SPEED);
                break;
            }
            case SWITCH_MID:
            {
                chassis_.SetTargetVelocityRotation(0);
                reload_.SetTargetReloadRotation(0);
                break;
            }
            case SWITCH_DOWN:
            {
                // chassis_angle_diff = CalcYawError(remote_yaw_angle_ ,imu_.GetYawRadian());

                // chassis_.chassis_follow_pid_.SetTarget(0);
                // chassis_.chassis_follow_pid_.SetNow(chassis_angle_diff);
                // chassis_.chassis_follow_pid_.CalculatePeriodElapsedCallback();

                // chassis_.SetTargetVelocityRotation(chassis_.chassis_follow_pid_.GetOut());
                reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);

                break;
            }
            default:
            {
                chassis_.SetTargetVelocityRotation(0);
                break;
            }
        }
        

        /****************************   Supercap   ****************************/


        if(mcu_comm_data_local.supercap){
            supercap_.SetSupercapCharge(SUPERCAP_SWITCH_STATUS_ENABLE);
        }else if(mcu_comm_data_local.supercap){
            supercap_.SetSupercapCharge(SUPERCAP_SWITCH_STATUS_DISABLE);
        }


        /****************************   Debug   ****************************/


        osDelay(pdMS_TO_TICKS(1));
    }
}



