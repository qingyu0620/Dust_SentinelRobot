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
 * @brief MCU通讯函数
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
     osThreadNew(McuComm::TaskEntry, this, &kMcuCommTaskAttr);
}

/**
 * @brief MCU任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void McuComm::TaskEntry(void *argument) {
     McuComm *self = static_cast<McuComm *>(argument);  // 还原 this 指针
     self->Task();  // 调用成员函数
}

/**
 * @brief MCU任务函数
 * 
 */
void McuComm::Task()
{
     McuCommData mcu_comm_data_local;
     for (;;)
     {    // 用临界区一次性复制，避免撕裂
          // __disable_irq();
          // mcu_comm_data__Local = *const_cast<const struct McuCommData*>(&(mcu_comm_data_));
          // __enable_irq();
          // osDelay(pdMS_TO_TICKS(10));
     }
}

/**
 * @brief MCU can发送命令函数
 * 
 */
void McuComm::CanSendCommand()
{
     static uint8_t can_tx_frame[8];
     // 第一帧发送底盘数据
     can_tx_frame[0] = mcu_chassis_data_.start_of_frame;
     can_tx_frame[1] = mcu_chassis_data_.chassis_speed_x >> 8;
     can_tx_frame[2] = mcu_chassis_data_.chassis_speed_x;
     can_tx_frame[3] = mcu_chassis_data_.chassis_speed_y >> 8;
     can_tx_frame[4] = mcu_chassis_data_.chassis_speed_y;
     can_tx_frame[5] = mcu_chassis_data_.chassis_rotation >> 8;
     can_tx_frame[6] = mcu_chassis_data_.chassis_rotation;
     can_tx_frame[7] = mcu_chassis_data_.chassis_spin;
     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);

     // 第二帧发送拨弹盘，yaw角数据
     can_tx_frame[0] = mcu_comm_data_.start_of_frame;
     can_tx_frame[1] = mcu_comm_data_.armor;
     can_tx_frame[2] = mcu_comm_data_.yaw >> 8;
     can_tx_frame[3] = mcu_comm_data_.yaw;
     can_tx_frame[4] = mcu_comm_data_.supercap;
     can_tx_frame[5] = mcu_comm_data_.switch_r;
     can_tx_frame[6] = 0x00;
     can_tx_frame[7] = 0x00;
     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief MCU can回调函数
 * 
 * @param rx_data 
 */
void McuComm::CanRxCpltCallback(uint8_t* rx_data)
{
     // 判断在线

     // 处理数据 , 解包
     switch (rx_data[0])
     {
         
     }
}
