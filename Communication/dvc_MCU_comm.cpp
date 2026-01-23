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

/* Private macros ------------------------------------------------------------*/

#define MAX_MCU_DISALIVE_PERIOD   5          // 50ms

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
          .stack_size = 256,
          .priority = (osPriority_t) osPriorityNormal
     };
     // 启动任务，将 this 传入
     osThreadNew(McuComm::TaskEntry, this, &kMcuCommTaskAttr);
}

/**
 * @brief McuComm任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void McuComm::TaskEntry(void *argument)
{
     McuComm *self = static_cast<McuComm *>(argument);  // 还原 this 指针
     self->Task();  // 调用成员函数
}

/**
 * @brief McuComm清理数据函数
 * 
 */
void McuComm::ClearData()
{
     recv_chassis_data_.chassis_speed_x = 1024;
     recv_chassis_data_.chassis_speed_y = 1024;
     recv_chassis_data_.rotation = 1024;

     recv_comm_data_.mouse_lr.all = 0;
     recv_comm_data_.keyboard.all = 0;
}

/**
 * @brief McuComm存活周期检测回调函数
 * 
 */
void McuComm::AlivePeriodElapsedCallback()
{
     if(++alive_count_ >= MAX_MCU_DISALIVE_PERIOD)
     {
          if(pre_flag_ == flag_)
          {
               mcu_alive_state_ = MCU_ALIVE_STATE_DISABLE;
               ClearData();
          }
          else
          {
               mcu_alive_state_ = MCU_ALIVE_STATE_ENABLE;
          }

          pre_flag_ = flag_;

          alive_count_ = 0;
     }
}

/**
 * @brief McuComm任务函数
 * 
 */
void McuComm::Task()
{
     for (;;)
     {
          AlivePeriodElapsedCallback();
          osDelay(pdMS_TO_TICKS(10));
     }
}

/**
 * @brief McuComm回调函数
 * 
 * @param rx_data 
 */
void McuComm::CanRxCpltCallback(uint8_t* rx_data)
{
     // 滑动窗口，检测是否在线
     flag_ += 1;
     
     DataProcess(rx_data);
}

/**
 * @brief McuComm数据处理函数
 * 
 * @param rx_data 
 */
void McuComm::DataProcess(uint8_t* rx_data)
{
     switch (rx_data[0])
     {
          case (0xAA): // 底盘包
          {
               recv_chassis_data_.chassis_speed_x      = rx_data[1] << 8 | rx_data[2];
               recv_chassis_data_.chassis_speed_y      = rx_data[3] << 8 | rx_data[4];
               recv_chassis_data_.rotation             = rx_data[5] << 8 | rx_data[6];
               recv_chassis_data_.switch_lr.all        = rx_data[7];

               break;
          }
          case (0xAB): // 拨弹盘，yaw角包
          {
               recv_comm_data_.mouse_lr.all = rx_data[1];

               recv_comm_data_.keyboard.all = rx_data[2] << 8 | rx_data[3];

               memcpy(&recv_comm_data_.imu_yaw.b, &rx_data[4], 4);

               break;
          }
          case (0xAC): // 自瞄yaw包
          {
               recv_autoaim_data_.mode = rx_data[1];

               memcpy(&recv_autoaim_data_.autoaim_yaw_ang, &rx_data[2], 4);

               break;
          }
     }
}
