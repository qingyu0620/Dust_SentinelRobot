/**
 * @file dvc_MCU_comm.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_MCU_comm.h"
#include "dvc_motor_dm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief McuComm通讯函数
 * 
 * @param hcan can句柄
 * @param can_rx_id 接收id
 * @param can_tx_id 发送id
 */
void McuComm::Init(CAN_HandleTypeDef* hcan, uint8_t can_rx_id, uint8_t can_tx_id)
{
     if (hcan->Instance == CAN1)
     {
          can_manage_object_ = &g_can1_manage_object;
     }
     else if (hcan->Instance == CAN2)
     {
          can_manage_object_ = &g_can2_manage_object;
     }

     can_rx_id_ = can_rx_id;
     can_tx_id_ = can_tx_id;

     static const osThreadAttr_t kMcuCommTaskAttr = {
          .name = "mcu_comm_task",
          .stack_size = 512,
          .priority = (osPriority_t) osPriorityNormal
     };
     // 启动任务，将 this 传入
     // osThreadNew(McuComm::TaskEntry, this, &kMcuCommTaskAttr);
}

/**
 * @brief McuComm任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void McuComm::TaskEntry(void *argument) {
     McuComm *self = static_cast<McuComm *>(argument);  // 还原 this 指针
     self->Task();  // 调用成员函数
}

/**
 * @brief McuComm任务函数
 * 
 */
void McuComm::Task()
{
     for (;;)
     {    // 用临界区一次性复制，避免撕裂
          // __disable_irq();
          // mcu_comm_data_local = *const_cast<const struct McuCommData*>(&(mcu_comm_data_));
          // __enable_irq();
          // osDelay(pdMS_TO_TICKS(10));
     }
}

/**
 * @brief McuComm发送命令函数
 * 
 */
void McuComm::CanSendCommand()
{
     static uint8_t can_tx_frame[8];
}

/**
 * @brief McuComm发送自瞄函数
 * 
 */
void McuComm::CanSendAutoaim()
{
     static uint8_t can_tx_frame[8];

     can_tx_frame[0] = send_autoaim_data_.start_of_yaw_frame;

     memcpy(&can_tx_frame[1], send_autoaim_data_.autoaim_yaw.b, 4);

     can_tx_frame[5] = 0x00;
     can_tx_frame[6] = 0x00;
     can_tx_frame[7] = 0x00;

     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief McuComm回调函数
 * 
 * @param rx_data 
 */
void McuComm::CanRxCpltCallback(uint8_t* rx_data)
{
     // 判断在线

     // 处理数据 , 解包
     switch (rx_data[0])
     {
          case (0xAA): // 底盘包
          {
               recv_chassis_data_.chassis_speed_x      = rx_data[1] << 8 | rx_data[2];
               recv_chassis_data_.chassis_speed_y      = rx_data[3] << 8 | rx_data[4];
               recv_chassis_data_.rotation             = rx_data[5] << 8 | rx_data[6];

               switch(rx_data[7])
               {
                    case 1:
                    recv_chassis_data_.switch_l = CHASSIS_SPIN_CLOCKWISE;
                    break;
                    case 3:
                    recv_chassis_data_.switch_l = CHASSIS_SPIN_DISABLE;
                    break;
                    case 2:
                    recv_chassis_data_.switch_l = CHASSIS_SPIN_COUNTER_CLOCK_WISE;
                    break;
                    default:
                    recv_chassis_data_.switch_l = CHASSIS_SPIN_DISABLE;
                    break;
               }
               
               break;
          }
          case (0xAB): // 拨弹盘，yaw角包
          {
               recv_comm_data_.armor                = rx_data[1];
               recv_comm_data_.supercap             = rx_data[2];

               switch(rx_data[3])
               {
                    case (1):
                    {
                         recv_comm_data_.switch_r = Switch_UP;
                         break;
                    }
                    case (3):
                    {
                         recv_comm_data_.switch_r = Switch_MID;
                         break;
                    }
                    case (2):
                    {
                         recv_comm_data_.switch_r = Switch_DOWN;
                         break;
                    }
                    default:
                    {
                         recv_comm_data_.switch_r = Switch_MID;
                         break;
                    }
               }

               recv_comm_data_.imu_yaw.b[0]      = rx_data[4];
               recv_comm_data_.imu_yaw.b[1]      = rx_data[5];
               recv_comm_data_.imu_yaw.b[2]      = rx_data[6];
               recv_comm_data_.imu_yaw.b[3]      = rx_data[7];

               break;
          }
          case (0xAC): // 自瞄yaw包
          {
               recv_autoaim_data_.autoaim_yaw.b[0]     = rx_data[1];
               recv_autoaim_data_.autoaim_yaw.b[1]     = rx_data[2];
               recv_autoaim_data_.autoaim_yaw.b[2]     = rx_data[3];
               recv_autoaim_data_.autoaim_yaw.b[3]     = rx_data[4];

               break;
          }
     }

}
