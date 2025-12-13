/**
 * @file dvc_PC_comm.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_PC_comm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief PcComm初始化函数
 * 
 */
void PcComm::Init()
{
    dwt_init(168);

    static const osThreadAttr_t KPcCommTaskAttr = 
    {
        .name = "pccomm_task",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(PcComm::TaskEntry, this, &KPcCommTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void PcComm::TaskEntry(void *argument)
{
    PcComm *self = static_cast<PcComm *>(argument);
    self->Task();
}

/**
 * @brief PcComm更新自瞄数据函数
 * 
 */
void PcComm::UpdataAutoaimData()
{
    memcpy(&send_autoaim_data.q, INS.q, 16);

    send_autoaim_data.mode           = 0;
    send_autoaim_data.yaw.ang        = INS.Yaw;
    send_autoaim_data.yaw.vel        = INS.Gyro[Z];
    send_autoaim_data.pitch.ang      = -INS.Roll;
    send_autoaim_data.pitch.vel      = -INS.Gyro[X]; 
    send_autoaim_data.bullet.speed   = 16;
    send_autoaim_data.bullet.count   = 20;
}


/**
 * @brief PcComm发送信息函数
 * 
 */
void PcComm::Send_Message()
{
    uint16_t lenth = sizeof(send_autoaim_data);
    uint8_t buffer[lenth];  // 明确的43字节

    send_autoaim_data.crc16 = 0;

    memcpy(buffer, &send_autoaim_data, lenth);
    append_crc16_check_sum(buffer, lenth);
    
    usb_transmit(buffer, lenth);
}

/**
 * @brief PcComm任务函数
 * 
 */
void PcComm::Task()
{
    for(;;)
    {
        UpdataAutoaimData();
        Send_Message();
        osDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief PcComm接收回调函数
 * 
 */
void PcComm::RxCpltCallback()
{
    if(bsp_usb_rx_buffer[0] == 'S' && bsp_usb_rx_buffer[1] == 'P')
    {
        uint16_t lenth = sizeof(recv_autoaim_data);
        memcpy(&recv_autoaim_data, bsp_usb_rx_buffer, lenth);
        
        // if(verify_crc16_check_sum(bsp_usb_rx_buffer, lenth))
        // {
        //     memcpy(&recv_autoaim_data, bsp_usb_rx_buffer, lenth);
        // }
    }
    else if(bsp_usb_rx_buffer[0] == recv_navigation_data.start_of_frame)
    {
        memcpy(recv_navigation_data.linear_x, &bsp_usb_rx_buffer[1], 4);
        memcpy(recv_navigation_data.linear_y, &bsp_usb_rx_buffer[5], 4);

        recv_navigation_data.crc16[0] = bsp_usb_rx_buffer[9];
        recv_navigation_data.crc16[1] = bsp_usb_rx_buffer[10];
    }
}
