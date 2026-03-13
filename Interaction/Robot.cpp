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
#include "app_shoot.h"
#include "cmsis_os2.h"
#include "dvc_MCU_comm.h"
#include "dvc_PC_comm.h"
#include "dvc_remote_dr16.h"
#include <cstdio>

/* Private macros ------------------------------------------------------------*/

#define INTERVAL_LIMIT(data, max, min)      \
    do{                                     \
         if((data) >= (max)){               \
            (data) = (max);                 \
        } else if((data) <= (min)){         \
            (data) = (min);                 \
        }}while(0)

#define SCAN_PITCH_RATIO        400.f
#define AUTOAIM_PITCH_RATIO     150.f

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

    static uint16_t print_count = 0;  // 打印计数器，每10ms打印一次

    for(;;)
    {
        /****************************   Robot    ****************************/


        if(remote_dr16_.output_.remote.switch_r == SWITCH_MID)
        {
            SetControlMethod(ROBOT_CONTROL_METHOD_REMOTE);
        }
        else if(remote_dr16_.output_.remote.switch_r == SWITCH_DOWN)
        {
            SetControlMethod(ROBOT_CONTROL_METHOD_NAVIGATION);
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


        // 自瞄模式
        if(remote_dr16_.output_.remote.switch_r == SWITCH_UP)
        {
            switch (pc_comm_.recv_auto_data.mode)
            {
                case (PC_AUTOAIM_MODE_IDLE):
                {
                    remote_radian = remote_dr16_.output_.remote.pitch;

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                case (PC_AUTOAIM_MODE_REAR):
                {
                    scan_pitch_count_ += M_PI / SCAN_PITCH_RATIO;
                    scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                    remote_radian = MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_);

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

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

                        shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);
                    }
                    else
                    {
                        float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_auto_data.pitch_ang);

                        remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;

                        shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                        scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始
                    }

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    break;
                }
            }
        }
        // 正常遥控控制
        else if (remote_dr16_.output_.remote.switch_r == SWITCH_MID)
        {
            remote_radian = remote_dr16_.output_.remote.pitch;

            INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

            gimbal_.SetTargetPitchRadian(remote_radian);

            shoot_.SetTargetShootOmega(0);

            scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始
            
        }
        // 巡航模式
        else if (remote_dr16_.output_.remote.switch_r == SWITCH_DOWN)
        {
            if (pc_comm_.recv_auto_data.scan_status == true)
            {
                switch (pc_comm_.recv_auto_data.mode)
                {
                    case (PC_AUTOAIM_MODE_IDLE):
                    {
                        scan_pitch_count_ += M_PI / SCAN_PITCH_RATIO;
                        scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                        remote_radian = MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_);

                        INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                        gimbal_.SetTargetPitchRadian(remote_radian);

                        shoot_.SetTargetShootOmega(0);

                        break;
                    }
                    case (PC_AUTOAIM_MODE_REAR):
                    {
                        scan_pitch_count_ += M_PI / SCAN_PITCH_RATIO;
                        scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                        remote_radian = MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_);

                        INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                        gimbal_.SetTargetPitchRadian(remote_radian);

                        shoot_.SetTargetShootOmega(0);

                        break;
                    }
                    case (PC_AUTOAIM_MODE_FRONT):
                    {
                        float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_auto_data.pitch_ang);

                        remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;

                        INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                        gimbal_.SetTargetPitchRadian(remote_radian);

                        // shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                        scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始
                        
                        break;
                    }
                }
            }
            else
            {
                remote_radian = remote_dr16_.output_.remote.pitch;

                INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                gimbal_.SetTargetPitchRadian(remote_radian);

                shoot_.SetTargetShootOmega(0);

                scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始
            }
        }

        gimbal_.SetNowImuPitchRadian(normalize_angle_pm_pi(imu_.GetRollAngle()));


        /****************************   Debug   ****************************/

        printf("%d,%d\n", pc_comm_.recv_auto_data.mode, pc_comm_.GetAutoAimStatus());

        
        osDelay(pdMS_TO_TICKS(1));
    }
}
