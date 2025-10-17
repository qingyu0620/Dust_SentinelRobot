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
#include "app_chassis.h"
#include "app_gimbal.h"
#include "dvc_remote_dji.h"
#include "bsp_can.h"
#include "bsp_uart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
RemoteDjiDR16 dr16;
Chassis sentinel;
Gimbal gimbal;


/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

void can1_callback_function(CanRxBuffer* CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId) 
    {
        case (0x201):
        {
            sentinel.motor_chassis_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            sentinel.motor_chassis_2_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x203):
        {
            sentinel.motor_chassis_3_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x204):
        {
            sentinel.motor_chassis_4_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        default:
            break;
    }
}

void uart3_callback_function(uint8_t* buffer, uint16_t length)
{	
	dr16.DbusTransformation(buffer);
}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    uart_init(&huart3, uart3_callback_function, UART_BUFFER_LENGTH);
    can_init(&hcan1, can1_callback_function);
    sentinel.Init();
}
