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
#include "app_shoot.h"
#include "bsp_uart.h"

#include "dvc_remote_dji.h"
#include "dvc_MCU_comm.h"

#include "app_gimbal.h"



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
        default:
            break;
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
        case (0x04):
        {
            robot_.gimbal_.motor_yaw_.CanRxCpltCallback(CAN_RxMessage->data);
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
        default:
            break;
    }
}

/**
 * @brief dbus回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart3_callback_function(uint8_t* buffer, uint16_t length) 
{	
	robot_.remote_dr16_.DbusTransformation(buffer);
    robot_.mcu_comm_.mcu_comm_data_.start_of_frame = 0xAB;
    robot_.mcu_comm_.mcu_comm_data_.chassis_speed_x  = robot_.remote_dr16_.output.chassis_x;
    robot_.mcu_comm_.mcu_comm_data_.chassis_speed_y  = robot_.remote_dr16_.output.chassis_y;
    robot_.mcu_comm_.mcu_comm_data_.chassis_rotation = robot_.remote_dr16_.output.chassis_r;
}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    // 下板通讯
    can_init(&hcan1, can1_callback_function);
    // 摩擦轮   pitch角电机
    can_init(&hcan2, can2_callback_function);
    robot_.Init();
}
