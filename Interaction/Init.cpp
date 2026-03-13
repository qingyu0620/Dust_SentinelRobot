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
#include "bsp_uart.h"
#include "usart.h"

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
void can1_callback_function(CanRxBuffer* CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId) 
    {
        case (0x201):
        {
            robot_.chassis_.motor_chassis_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            robot_.chassis_.motor_chassis_2_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x203):
        {
            robot_.chassis_.motor_chassis_3_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x204):
        {
            robot_.chassis_.motor_chassis_4_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x205):
        {
            robot_.reload_.motor_reload_1_.CanRxCpltCallback(CAN_RxMessage->data);
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
    switch(CAN_RxMessage->header.StdId)
    {
        case (0x01):
        {
            robot_.mcu_comm_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x07):
        {
            robot_.gimbal_.motor_yaw_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x030):
        {
            robot_.supercap_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
    }
}

/**
 * @brief UART3回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart3_callback_function(uint8_t* buffer, uint16_t length) 
{
	
}

/**
 * @brief UART6回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart6_callback_function(uint8_t* buffer, uint16_t length)
{	
    robot_.referee_.UartRxCpltCallback(buffer, length);
    
    robot_.mcu_comm_.send_fast_data_.bullet_number = robot_.referee_.Get17mmRemaining();
    robot_.mcu_comm_.send_fast_data_.bullet_speed.f = robot_.referee_.GetShootSpeed();
    robot_.mcu_comm_.send_fast_data_.middle_buff_status = robot_.referee_.GetCenterRfidStatus();

    robot_.mcu_comm_.send_slow_data_.stage_enum = robot_.referee_.GetGameStage();
    robot_.mcu_comm_.send_slow_data_.stage_remain_time = robot_.referee_.GetRemainingTime();
    robot_.mcu_comm_.send_slow_data_.robot_hp = robot_.referee_.GetHP(CURRENT_ROBOT_ID);
    robot_.mcu_comm_.send_slow_data_.is_jam = robot_.reload_.GetReloadState();
}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    can_init(&hcan1, can1_callback_function);
    can_init(&hcan2, can2_callback_function);

    uart_init(&huart1, nullptr, UART_BUFFER_LENGTH);
    robot_.Init();
}
