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

#include "bsp_uart.h"

#include "dvc_remote_dji.h"

#include "app_chassis.h"
#include "app_gimbal.h"



/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
RemoteDjiDR16 remote_dr16;
Chassis chassis_;
Gimbal gimbal_;


/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

void can1_callback_function(CanRxBuffer* CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId) 
    {
        case (0x201):
        {
            chassis_.motor_chassis_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            chassis_.motor_chassis_2_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x203):
        {
            chassis_.motor_chassis_3_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x204):
        {
            chassis_.motor_chassis_4_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        default:
            break;
    }
}

void can2_callback_function(CanRxBuffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId)
    {
        case (0x12)://01
        {
            gimbal_.motor_yaw_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x11)://02
        {
            gimbal_.motor_pitch_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        default:
            break;
    }
}

void uart3_callback_function(uint8_t* buffer, uint16_t length)
{	
	remote_dr16.DbusTransformation(buffer);
    chassis_.SetTargetVelocityX(remote_dr16.remotedata.x);
    chassis_.SetTargetVelocityY(remote_dr16.remotedata.y);
    chassis_.SetTargetVelocityRotation(remote_dr16.remotedata.r);
}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    uart_init(&huart3, uart3_callback_function, UART_BUFFER_LENGTH);
    can_init(&hcan1, can1_callback_function);
    // gimbal_.Init();
    chassis_.Init();
    
}
