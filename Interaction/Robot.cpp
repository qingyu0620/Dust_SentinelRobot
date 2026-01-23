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

#define INTERVAL_LIMIT(data, max, min)      \
    do{                                     \
         if((data) >= (max)){               \
            (data) = (max);                 \
        } else if((data) <= (min)){         \
            (data) = (min);                 \
        }}while(0)

#define SCAN_PITCH_RADIO        400
#define AUTOAIM_PITCH_RATIO     300.f

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

    // 等待云台yaw角回正
    osDelay(pdMS_TO_TICKS(5000));

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


        // 若掉线发送空白数据
        if(remote_dr16_.remote_dji_alive_status == REMOTE_DJI_STATUS_DISABLE)
        {
            mcu_comm_.ClearData();
        }

        mcu_comm_.UpdateAutoaimData(&pc_comm_.recv_autoaim_data);

        mcu_comm_.CanSendAutoaimData();


        /****************************   PcComm   ****************************/



        
        
        /****************************   Gimbal   ****************************/


        // 自瞄模式
        if(remote_dr16_.output_.remote.switch_r == SWITCH_UP || remote_dr16_.output_.mouse.press_r == REMOTE_DR16_KEY_STATUS_PRESS)
        {
            switch (pc_comm_.recv_autoaim_data.mode)
            {
                case(PC_AUTOAIM_MODE_IDLE):
                {
                    remote_radian = remote_dr16_.output_.remote.pitch + remote_dr16_.output_.mouse.mouse_y;

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    break;
                }
                case(PC_AUTOAIM_MODE_REAR):
                {
                    remote_radian = remote_dr16_.output_.remote.pitch + remote_dr16_.output_.mouse.mouse_y;

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);
                    
                    break;
                }
                case(PC_AUTOAIM_MODE_FRONT):
                {
                    float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_autoaim_data.pitch.pitch_ang);

                    remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始
                    
                    break;
                }
            }
        }
        // 巡航模式
        else if(remote_dr16_.output_.remote.switch_r == SWITCH_DOWN || remote_dr16_.output_.keyboard.keycode.q == REMOTE_DR16_KEY_STATUS_PRESS)
        {
            switch (pc_comm_.recv_autoaim_data.mode)
            {
                case(PC_AUTOAIM_MODE_IDLE):
                {
                    scan_pitch_count_ += M_PI / SCAN_PITCH_RADIO;
                    scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                    remote_radian = MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_);

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    // shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                case(PC_AUTOAIM_MODE_REAR):
                {
                    scan_pitch_count_ += M_PI / SCAN_PITCH_RADIO;
                    scan_pitch_count_ = normalize_pi(scan_pitch_count_);
                    remote_radian = MIN_PITCH_RADIAN * arm_sin_f32(scan_pitch_count_);

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    // shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                case(PC_AUTOAIM_MODE_FRONT):
                {
                    float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_autoaim_data.pitch.pitch_ang);

                    remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    // shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始
                    
                    break;
                }
            }
        }
        // 正常模式
        else
        {
            remote_radian = remote_dr16_.output_.remote.pitch + remote_dr16_.output_.mouse.mouse_y;

            INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

            gimbal_.SetTargetPitchRadian(remote_radian);

            shoot_.SetTargetShootOmega(0);

            scan_pitch_count_ = 0;      // 累加数据清零，下一次扫描从头开始
        }

        gimbal_.SetNowImuPitchRadian(normalize_angle_pm_pi(imu_.GetRollAngle()));


        if(remote_dr16_.output_.remote.switch_r == SWITCH_UP || 
           remote_dr16_.output_.mouse.press_r || remote_dr16_.output_.keyboard.keycode.e)
        {
            shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);
        }
        else
        {
            shoot_.SetTargetShootOmega(0);
        }


        /****************************   Debug   ****************************/


        osDelay(pdMS_TO_TICKS(1));
    }
}
