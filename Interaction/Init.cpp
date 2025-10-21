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

#include "app_shoot.h"
#include "bsp_uart.h"

#include "dvc_remote_dji.h"

#include "app_gimbal.h"



/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

RemoteDjiDR16 remote_dr16_;
Gimbal gimbal_;
Shoot shoot_;

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
            gimbal_.motor_yaw_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x201):
        {
            shoot_.motor_shoot_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            shoot_.motor_shoot_2_.CanRxCpltCallback(CAN_RxMessage->data);
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
	remote_dr16_.DbusTransformation(buffer);
    gimbal_.SetTargetPitchAngle(remote_dr16_.output.y);
}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    uart_init(&huart3, uart3_callback_function, UART_BUFFER_LENGTH);
    can_init(&hcan1, can1_callback_function);
    can_init(&hcan2, can2_callback_function);
    gimbal_.Init();
    // shoot_.Init();
}
