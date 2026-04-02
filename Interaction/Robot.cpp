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
#include "dvc_referee.h"

/* Private macros ------------------------------------------------------------*/

constexpr float SCAN_PITCH_RATIO     =  375.f;
constexpr float AUTOAIM_PITCH_RATIO  =  150.f;

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

    // 遥控初始化
    remote_dr16_.Init(&huart3, uart3_callback_function, UART_BUFFER_LENGTH);

    // 陀螺仪初始化
    imu_.Init();

    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan1, 0x00, 0x01);

    // 上位机通讯
    pc_comm_.Init();

    // 云台初始化
    gimbal_.Init();

    // 摩擦轮初始化
    shoot_.Init();
    
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
 * @brief Robot设定导航状态函数
 * 
 * @param robot_navigation_state 
 */
void Robot::SetControlMethod(RobotControlMethod robot_control_method)
{
    robot_control_method_ = robot_control_method;
}

/**
 * @brief Robot任务函数
 * 
 */
void Robot::Task()
{
    McuRecvRefereeFastData mcu_fast_data_local;
    mcu_fast_data_local.bullet_number = 0;
    mcu_fast_data_local.bullet_speed.f = 0.f;
    mcu_fast_data_local.middle_buff_status = 0;

    McuRecvRefereeSlowData mcu_slow_data_local;
    mcu_slow_data_local.stage_enum = 0;
    mcu_slow_data_local.stage_remain_time = 0;
    mcu_slow_data_local.robot_hp = 0;

    for(;;)
    {
        /****************************   Robot    ****************************/


        if(remote_dr16_.output_.remote.switch_r == SWITCH_DOWN) {
            SetControlMethod(ROBOT_CONTROL_METHOD_NAVIGATION);
        } else {
            SetControlMethod(ROBOT_CONTROL_METHOD_REMOTE);
        }


        /****************************   McuComm   ****************************/

        
        __disable_irq();
        mcu_fast_data_local = *(static_cast<const McuRecvRefereeFastData*>(&(mcu_comm_.recv_fast_data_)));
        mcu_slow_data_local = *(static_cast<const McuRecvRefereeSlowData*>(&(mcu_comm_.recv_slow_data_)));
        __enable_irq();

        // 如果断开连接，清理数据
        if(remote_dr16_.remote_alive_status == REMOTE_ALIVE_STATUS_DISABLE) {
            mcu_comm_.ClearRemoteData(); 
        }

        if (pc_comm_.GetAlivState() == PC_ALIVE_STATE_DISABLE) 
        {
            // 若此时为导航模式
            if (remote_dr16_.output_.remote.switch_r == SWITCH_DOWN) 
            {
                mcu_comm_.send_chassis_data_.chassis_speed_x = 1024;
                mcu_comm_.send_chassis_data_.chassis_speed_y = 1024;
            }
            mcu_comm_.ClearAutoData();
        }

        mcu_comm_.CanSendAutoData();


        /****************************   PcComm   ****************************/


        pc_comm_.UpdataAutoaimData(mcu_fast_data_local, mcu_slow_data_local);


        /****************************   Gimbal   ****************************/


        // 自瞄测试模式
        if (remote_dr16_.output_.remote.switch_r == SWITCH_UP)
        {
            switch (pc_comm_.recv_auto_data.mode)
            {
                case (PC_AUTOAIM_MODE_IDLE):
                {
                    scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始

                    remote_radian = std::clamp(remote_dr16_.output_.remote.pitch, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                case (PC_AUTOAIM_MODE_REAR):
                {
                    scan_pitch_count_ += M_PI / SCAN_PITCH_RATIO;
                    scan_pitch_count_ = normalize_pi(scan_pitch_count_);

                    remote_radian = std::clamp(MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_), MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                // 自瞄优先
                case (PC_AUTOAIM_MODE_FRONT):
                {
                    if (mcu_slow_data_local.is_jam)
                    {
                        scan_pitch_count_ += M_PI / SCAN_PITCH_RATIO;
                        scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                        remote_radian = MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_);
                    }
                    else
                    {
                        scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始

                        float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_auto_data.pitch_ang);

                        remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;

                        shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);
                    }

                    remote_radian = std::clamp(remote_radian, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
            }
        }
        else if (remote_dr16_.output_.remote.switch_r == SWITCH_MID)
        {
            scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始

            remote_radian = std::clamp(remote_dr16_.output_.remote.pitch, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

            gimbal_.SetTargetPitchRadian(remote_radian);

            shoot_.SetTargetShootOmega(0);
        }
        // 巡航模式
        else if (mcu_slow_data_local.stage_enum == Referee_Game_Status_Stage_BATTLE)
        {
            switch (pc_comm_.recv_auto_data.mode)
            {
                case (PC_AUTOAIM_MODE_IDLE):
                {
                    scan_pitch_count_ += M_PI / SCAN_PITCH_RATIO;
                    scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                    remote_radian = std::clamp(MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_), MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                case (PC_AUTOAIM_MODE_REAR):
                {
                    scan_pitch_count_ += M_PI / SCAN_PITCH_RATIO;
                    scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                    remote_radian = std::clamp(MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_), MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                case (PC_AUTOAIM_MODE_FRONT):
                {
                    if (mcu_slow_data_local.is_jam)
                    {
                        scan_pitch_count_ += M_PI / SCAN_PITCH_RATIO;
                        scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                        remote_radian = MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_);
                    }
                    else
                    {
                        float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_auto_data.pitch_ang);
                        remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;
                        remote_radian = std::clamp(remote_radian, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

                        scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始
                    }

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
            }
        } 
        

        gimbal_.SetNowImuPitchRadian(normalize_angle_pm_pi(imu_.GetRollAngle()));


        /****************************   Debug   ****************************/


        osDelay(pdMS_TO_TICKS(1));
    }
}
