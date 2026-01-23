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

#define MAX_MCU_DISALIVE_PERIOD   10

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief McuComm初始化函数
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
void McuComm::TaskEntry(void *argument)
{
     McuComm *self = static_cast<McuComm *>(argument);  // 还原 this 指针
     self->Task();  // 调用成员函数
}

/**
 * @brief McuComm更新自瞄数据
 * 
 * @param pc_recv_autoaim_data 
 */
void McuComm::UpdateAutoaimData(PCRecvAutoAimData* pc_recv_autoaim_data)
{
     send_autoaim_data_.mode = pc_recv_autoaim_data->mode;
     send_autoaim_data_.autoaim_yaw_angle.f = pc_recv_autoaim_data->yaw.yaw_ang;
     send_autoaim_data_.flag = pc_recv_autoaim_data->flag;
}

/**
 * @brief McuComm发送底盘数据函数
 * 
 */
void McuComm::CanSendChassisData()
{
     static uint8_t can_tx_frame[8];
     // 第一帧发送底盘数据
     can_tx_frame[0] = 0xAA;

     can_tx_frame[1] = send_chassis_data_.chassis_speed_x >> 8;
     can_tx_frame[2] = send_chassis_data_.chassis_speed_x;

     can_tx_frame[3] = send_chassis_data_.chassis_speed_y >> 8;
     can_tx_frame[4] = send_chassis_data_.chassis_speed_y;

     can_tx_frame[5] = send_chassis_data_.rotation >> 8;
     can_tx_frame[6] = send_chassis_data_.rotation;

     can_tx_frame[7] = send_chassis_data_.switch_lr.all;

     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief McuComm发送命令数据函数
 * 
 */
void McuComm::CanSendCommandData()
{
     static uint8_t can_tx_frame[8];
     static McuConv yaw_conv;

     yaw_conv.f = INS.Yaw;

     can_tx_frame[0] = 0xAB;
     can_tx_frame[1] = send_command_data_.mouse_lr.all;
     can_tx_frame[2] = send_command_data_.keyboard.all >> 8;
     can_tx_frame[3] = send_command_data_.keyboard.all;

     memcpy(&can_tx_frame[4], yaw_conv.b, 4);

     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief McuComm发送自瞄数据函数
 * 
 */
void McuComm::CanSendAutoaimData()
{
     static uint8_t can_tx_frame[8];

     can_tx_frame[0] = 0xAC;
     can_tx_frame[1] = send_autoaim_data_.mode;

     memcpy(&can_tx_frame[2], &send_autoaim_data_.autoaim_yaw_angle, 4);

     can_tx_frame[6] = send_autoaim_data_.flag;
     can_tx_frame[7] = 0x00;

     can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief McuComm掉线数据函数
 * 
 */
void McuComm::ClearData()
{
     send_chassis_data_.chassis_speed_x = 1024;
     send_chassis_data_.chassis_speed_y = 1024;
     send_chassis_data_.rotation = 1024;

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
     for(;;)
     {    
          CanSendChassisData();
          CanSendCommandData();
          osDelay(pdMS_TO_TICKS(1));
     }
}

/**
 * @brief McuComm回调函数
 * 
 * @param rx_data 
 */
void McuComm::CanRxCpltCallback(uint8_t* rx_data)
{
     // 滑动窗口，判断是否在线
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
     // 处理数据 , 解包
     switch (rx_data[0])
     {
         
     }
}
