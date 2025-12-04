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

#define MAX_SHOOT_SPEED             50.f

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
 * @brief Robot任务函数
 * 
 */
void Robot::Task()
{
    for(;;)
    {
        /****************************   MCUcomm   ****************************/


        // 若掉线发送空白数据
        if(remote_dr16_.remote_dji_alive_status == REMOTE_DJI_STATUS_DISABLE)
        {
            mcu_comm_.DisconnectData();
        }
        mcu_comm_.UpdataAutoaimData(&pc_comm_.recv_autoaim_data);


        /****************************   PCcomm   ****************************/

        

        
        /****************************   云台   ****************************/


        if(pc_comm_.recv_autoaim_data.mode == 0){
            gimbal_.SetTargetPitchRadian(remote_dr16_.output_.pitch);
            gimbal_.SetControlPitch(0, 0);
        }else if(pc_comm_.recv_autoaim_data.mode == 1){
            gimbal_.SetControlPitch(pc_comm_.recv_autoaim_data.pitch.pitch_vel, pc_comm_.recv_autoaim_data.pitch.pitch_acc);
        }


        /****************************   摩擦轮   ****************************/


        // 右按钮
        switch (remote_dr16_.output_.switch_r)
        {
            case SWITCH_UP:
            {
                // shoot_.SetTargetShootSpeed(MAX_SHOOT_SPEED);
                break;
            }
            case SWITCH_MID:
            {
                shoot_.SetTargetShootSpeed(0);
                break;
            }
            default:
            {
                shoot_.SetTargetShootSpeed(0);
                break;
            }
        }


        /****************************   调试   ****************************/

    
        osDelay(pdMS_TO_TICKS(1));
    }
}



