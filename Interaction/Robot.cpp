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

constexpr float K_NORM              = 1.f / 660.f;
constexpr float C_NORM              = -256.f / 165.f;

constexpr float REMOTE_YAW_RATIO    = 0.5f;
constexpr float AUTOAIM_YAW_RATIO   = 120.f;
constexpr float SCAN_YAW_RATIO      = 0.065f;
constexpr float REAR_YAW_RATIO      = 0.08f;

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

    Timer print{20};
    
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
        else if (mcu_chassis_data_local.switch_lr.switch_r == SWITCH_MID)
        {
            remote_yaw_radian_ += (M_PI / 180.f * (K_NORM * mcu_chassis_data_local.rotation + C_NORM)) * REMOTE_YAW_RATIO;
            remote_yaw_radian_ = normalize_pi(remote_yaw_radian_);

            gimbal_.SetTargetYawRadian(remote_yaw_radian_);
        }
        else if (mcu_comm_.send_slow_data_.stage_enum == Referee_Game_Status_Stage_BATTLE)
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

        if (mcu_chassis_data_local.switch_lr.switch_l == SWITCH_UP)
        {
            gimbal_.motor_yaw_.CanSendSaveZero();
            osDelay(100);
        };


        /****************************   Chassis   ****************************/


        chassis_.SetNowYawRadianDiff(-gimbal_.GetNowYawRadian());

        // 设置目标映射速度
        chassis_.SetTargetVxInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_x + C_NORM) * chassis_.GetMaxOmegaSpeed());
        chassis_.SetTargetVyInGimbal((K_NORM * mcu_chassis_data_local.chassis_speed_y + C_NORM) * chassis_.GetMaxOmegaSpeed());

        if (mcu_chassis_data_local.switch_lr.switch_l == SWITCH_DOWN && mcu_chassis_data_local.switch_lr.switch_r == SWITCH_DOWN)
        {
            if (mcu_comm_.send_slow_data_.robot_hp <= 380 || mcu_comm_.send_fast_data_.middle_buff_status || mcu_auto_data_local.chassis_mode)
            {
                chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_SPIN);
            }
            else
            {
                chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_NORMAL);
            }
        }
        else if (mcu_chassis_data_local.switch_lr.switch_l == SWITCH_DOWN && mcu_chassis_data_local.switch_lr.switch_r == SWITCH_MID) 
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_SPIN);
        }
        else
        {
            chassis_.SetChassisOperationMode(CHASSIS_OPERATION_MODE_NORMAL);
        }

        
        /*****************************     Reload   *****************************/


        if (mcu_chassis_data_local.switch_lr.switch_l == SWITCH_DOWN && mcu_chassis_data_local.switch_lr.switch_r == SWITCH_DOWN)
        {
            if (mcu_auto_data_local.fire) {
                reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);
            } else {
                reload_.SetTargetReloadRotation(0);
            }
        }
        else if (mcu_chassis_data_local.switch_lr.switch_l == SWITCH_DOWN && mcu_chassis_data_local.switch_lr.switch_r == SWITCH_UP)
        {
            reload_.SetTargetReloadRotation(MAX_RELOAD_SPEED);
        }
        else
        {
            reload_.SetTargetReloadRotation(0);
        }
    

        /****************************   Debug   ****************************/

        // print.Clock([&]()
        // {
        //     printf("%d\n", referee_.GetChassisEnergyBuffer());
        // });


        osDelay(pdMS_TO_TICKS(1));
    }
}



