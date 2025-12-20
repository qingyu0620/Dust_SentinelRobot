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
#include "dvc_MCU_comm.h"
#include "dvc_remote_dji.h"

/* Private macros ------------------------------------------------------------*/

#define INTERVAL_LIMIT(data, max, min)      \
    do{                                     \
         if((data) >= (max)){               \
            (data) = (max);                 \
        } else if((data) <= (min)){         \
            (data) = (min);                 \
        }}while(0)

#define MAX_SHOOT_SPEED             50.f
#define MAX_PITCH_RADIAN            0.45f
#define MIN_PITCH_RADIAN           -0.35f


/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Robot初始化函数
 * 
 */
void Robot::Init()
{
    // 遥控初始化
    // remote_vt03_.Init(&huart1, uart1_callback_function, UART_BUFFER_LENGTH);

    remote_dr16_.Init(&huart3, uart3_callback_function, UART_BUFFER_LENGTH);

    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan1, 0x00, 0x01);

    // 上位机通讯
    pc_comm_.Init();

    // 等待云台yaw角回正
    osDelay(pdMS_TO_TICKS(5000));

    // 陀螺仪初始化
    imu_.Init();

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
void Robot::SetNavigationState(RobotNavigationState robot_navigation_state)
{
    robot_navigation_state_ = robot_navigation_state;
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

        if(remote_dr16_.output_.switch_r == SWITCH_MID)
        {
            SetNavigationState(ROBOT_NAVIGATION_CLOSE);
        }
        else if(remote_dr16_.output_.switch_r == SWITCH_DOWN)
        {
            SetNavigationState(ROBOT_NAVIGATION_OPEN);
        }


        /****************************   MCUcomm   ****************************/


        // 若掉线发送空白数据
        if(remote_dr16_.remote_dji_alive_status == REMOTE_DJI_STATUS_DISABLE)
        {
            mcu_comm_.DisConnectData();
        }

        mcu_comm_.UpdataAutoaimData(&pc_comm_.recv_autoaim_data);

        mcu_comm_.CanSendAutoaimData();


        /****************************   PCcomm   ****************************/

        

        
        /****************************   云台   ****************************/

        // remote_angle = remote_dr16_.output_.pitch;

        // gimbal_.SetTargetPitchRadian(remote_angle);

        if(remote_dr16_.output_.switch_r == SWITCH_MID)
        {
            remote_angle = remote_dr16_.output_.pitch;
            gimbal_.SetTargetPitchRadian(remote_angle);
        }
        else if(remote_dr16_.output_.switch_r == SWITCH_UP)
        {
            switch (pc_comm_.recv_autoaim_data.mode)
            {
                case(AUTOAIM_MODE_IDIE):
                {
                    remote_angle = remote_dr16_.output_.pitch;
                    gimbal_.SetTargetPitchRadian(remote_angle);
                    break;
                }
                case(AUTOAIM_MODE_FOLLOW):
                {
                    float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_autoaim_data.pitch.pitch_ang);

                    remote_angle -= filtered_autoaim / 300.f;

                    INTERVAL_LIMIT(remote_angle, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_angle);

                    break;
                }
                case(AUTOAIM_MODE_FIRE):
                {
                    float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_autoaim_data.pitch.pitch_ang);

                    remote_angle -= filtered_autoaim / 300.f;

                    INTERVAL_LIMIT(remote_angle, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_angle);
                    // shoot_.SetTargetShootSpeed(MAX_SHOOT_SPEED);
                    break;
                }
            }
        }

        gimbal_.SetImuPitchAngle(normalize_angle_pm_pi(imu_.GetRollAngle()));


        /****************************   摩擦轮   ****************************/


        // 右按钮
        // switch (remote_dr16_.output_.switch_r)
        // {
        //     case SWITCH_UP:
        //     {
        //         shoot_.SetTargetShootSpeed(MAX_SHOOT_SPEED);
        //         break;
        //     }
        //     case SWITCH_MID:
        //     {
        //         shoot_.SetTargetShootSpeed(0);
        //         break;
        //     }
        //     default:
        //     {
        //         shoot_.SetTargetShootSpeed(0);
        //         break;
        //     }
        // }


        /****************************   调试   ****************************/
        

        osDelay(pdMS_TO_TICKS(1));
    }
}



