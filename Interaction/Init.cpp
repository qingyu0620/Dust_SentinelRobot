/**
 * @file Init.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "Init.h"
#include "Robot.h"
#include "dvc_MCU_comm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

Robot robot_;

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief can1回调函数
 * 
 * @param CAN_RxMessage 
 */
void can1_callback_function(CanRxBuffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId)
    {
        case (0x00):
        {
            robot_.mcu_comm_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
    }
}

/**
 * @brief can2回调函数
 * 
 * @param CAN_RxMessage 
 */
void can2_callback_function(CanRxBuffer* CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId) 
    {
        case (0x05):
        {
            robot_.gimbal_.motor_pitch_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x201):
        {
            robot_.shoot_.motor_shoot_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            robot_.shoot_.motor_shoot_2_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
    }
}

/**
 * @brief 
 * 
 * @param buffer 
 * @param length 
 */
void uart1_callback_function(uint8_t* buffer, uint16_t length)
{	
	
}

/**
 * @brief remote回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart3_callback_function(uint8_t* buffer, uint16_t length) 
{	
	robot_.remote_dr16_.UartRxCpltCallback(buffer);

    robot_.mcu_comm_.send_chassis_data_.start_of_frame   = 0xAA;
    
    if(robot_.robot_control_method_ == ROBOT_CONTROL_METHOD_REMOTE)
    {
        if(robot_.remote_dr16_.output_.keyboard.keycode.w)
        {
            robot_.mcu_comm_.send_chassis_data_.chassis_speed_x = 1684;
        }
        else if (robot_.remote_dr16_.output_.keyboard.keycode.s)
        {
            robot_.mcu_comm_.send_chassis_data_.chassis_speed_x = 364;
        }
        else
        {
            robot_.mcu_comm_.send_chassis_data_.chassis_speed_x = 1024;
        }

        if(robot_.remote_dr16_.output_.keyboard.keycode.a)
        {
            robot_.mcu_comm_.send_chassis_data_.chassis_speed_y = 364;
        }
        else if (robot_.remote_dr16_.output_.keyboard.keycode.d)
        {
            robot_.mcu_comm_.send_chassis_data_.chassis_speed_y = 1684;
        }
        else
        {
            robot_.mcu_comm_.send_chassis_data_.chassis_speed_y = 1024;
        }

        if(robot_.mcu_comm_.send_chassis_data_.chassis_speed_x == 1024)
        {
            robot_.mcu_comm_.send_chassis_data_.chassis_speed_x  = robot_.remote_dr16_.output_.remote.chassis_x;
        }
        if(robot_.mcu_comm_.send_chassis_data_.chassis_speed_y == 1024)
        {
            robot_.mcu_comm_.send_chassis_data_.chassis_speed_y  = robot_.remote_dr16_.output_.remote.chassis_y;
        }
    }

    robot_.mcu_comm_.send_chassis_data_.rotation = robot_.remote_dr16_.output_.remote.rotation -  robot_.remote_dr16_.output_.mouse.mouse_x + 1024;

    robot_.mcu_comm_.send_chassis_data_.switch_lr.switchcode.switch_l = robot_.remote_dr16_.output_.remote.switch_l;
    robot_.mcu_comm_.send_chassis_data_.switch_lr.switchcode.switch_r = robot_.remote_dr16_.output_.remote.switch_r;


    robot_.mcu_comm_.send_command_data_.start_of_frame     = 0xAB;

    robot_.mcu_comm_.send_command_data_.mouse_lr.mousecode.mouse_l = robot_.remote_dr16_.output_.mouse.mouse_lr.mousecode.mouse_l;
    robot_.mcu_comm_.send_command_data_.mouse_lr.mousecode.mouse_r = robot_.remote_dr16_.output_.mouse.mouse_lr.mousecode.mouse_r;
    robot_.mcu_comm_.send_command_data_.keyboard.all = robot_.remote_dr16_.output_.keyboard.all;
}

/**
 * @bief USB接收完成回调函数
 *
 * @param len 接收到的数据长度
 */
void usb_rx_callback(uint16_t len)
{
    robot_.pc_comm_.RxCpltCallback();

    if(robot_.robot_control_method_ == ROBOT_CONTROL_METHOD_NAVIGATION)
    {
        robot_.mcu_comm_.send_chassis_data_.chassis_speed_x  = robot_.pc_comm_.pc_chassis_x_;
        robot_.mcu_comm_.send_chassis_data_.chassis_speed_y  = robot_.pc_comm_.pc_chassis_y_;
    }
}

/**
 * @bief USB发送完成回调函数
 *
 * @param len 发送的数据长度
 */
void usb_tx_callback(uint16_t len)
{
    
}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    // 上位机通讯
    usb_init(usb_tx_callback, usb_rx_callback);
    // 下板通讯
    can_init(&hcan1, can1_callback_function);
    // 摩擦轮 pitch角电机
    can_init(&hcan2, can2_callback_function);
    
    robot_.Init();
}
