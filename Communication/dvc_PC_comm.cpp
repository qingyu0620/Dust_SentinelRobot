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

#define K_PC    660.f
#define C_PC    256.f / 165.f

#define MAX_PC_DISALIVE_PERIOD  200     // 200ms

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief PcComm初始化函数
 * 
 */
void PcComm::Init()
{
    static const osThreadAttr_t KPcCommTaskAttr = 
    {
        .name = "pccomm_task",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    // osThreadNew(PcComm::TaskEntry, this, &KPcCommTaskAttr);
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
    memcpy(&send_autoaim_data1.q, INS.q, 16);

    send_autoaim_data1.mode           = PC_AUTOAIM_MODE_IDLE;
    
    send_autoaim_data1.yaw.ang        = INS.Yaw;
    send_autoaim_data1.yaw.vel        = INS.Gyro[Z];

    send_autoaim_data1.pitch.ang      = -INS.Roll;
    send_autoaim_data1.pitch.vel      = -INS.Gyro[X]; 

    send_autoaim_data1.bullet.speed   = 16;
    send_autoaim_data1.bullet.count   = 20;


    send_autoaim_data2.start_of_frame = 0xA6;

    send_autoaim_data2.current_hp = 0;
    send_autoaim_data2.checksum = 0;
}

/**
 * @brief PcComm发送信息函数
 * 
 */
void PcComm::Send_Message()
{   
    uint16_t length1 = sizeof(send_autoaim_data1);
    static uint8_t buffer1[sizeof(send_autoaim_data1)];
    
    send_autoaim_data1.crc16 = 0;

    memcpy(buffer1, &send_autoaim_data1, length1);
    append_crc16_check_sum(buffer1, length1);

    usb_transmit(buffer1, length1);

    uint16_t length2 = sizeof(send_autoaim_data2);
    static uint8_t buffer2[sizeof(send_autoaim_data2)];

    memcpy(buffer2, &send_autoaim_data2, length2);
    // append_crc16_check_sum(buffer2, length2);
    
    usb_transmit(buffer2, length2);
}

/**
 * @brief PcComm清理数据函数
 * 
 */
void PcComm::ClearData()
{
    recv_autoaim_data.mode = PC_AUTOAIM_MODE_IDLE;

    recv_autoaim_data.yaw.yaw_ang = 0;
    recv_autoaim_data.yaw.yaw_vel = 0;
    recv_autoaim_data.yaw.yaw_acc = 0;

    recv_autoaim_data.pitch.pitch_ang = 0;
    recv_autoaim_data.pitch.pitch_acc = 0;
    recv_autoaim_data.pitch.pitch_vel = 0;

    recv_autoaim_data.flag = 1;

    recv_navigation_data.linear_x.f = 1024.f;
    recv_navigation_data.linear_y.f = 1024.f;
}

/**
 * @brief PcComm存活周期检测回调函数
 * 
 */
void PcComm::AlivePeriodElapsedCallback()
{
    if(++alive_beat_ >= MAX_PC_DISALIVE_PERIOD)
    {
        if(pre_flag_ == flag_)
        {
            pc_alive_state = PC_ALIVE_STATE_DISABLE;
            ClearData();
        }
        else
        {
            pc_alive_state = PC_ALIVE_STATE_ENABLE;
        }

        pre_flag_ = flag_;

        alive_beat_ = 0;
    }
}

/**
 * @brief PcComm判断自瞄状态函数
 * 
 * @param now_autoaim_status 当前自瞄状态
 * @param pre_autoaim_status 上一刻自瞄状态
 */
void PcComm::JudgeAutoaimStatus(PcAutoAimStatus* now_autoaim_status, PcAutoAimStatus pre_autoaim_status)
{
    // 获取当前收到的目标状态
    PcAutoAimStatus input = *now_autoaim_status;
    static uint16_t rear_beat = 0;      //后置摄像头运行节拍数

    switch (pre_autoaim_status)
    {
        case(PC_AUTOAIM_MODE_IDLE):
        {
            *now_autoaim_status = input;
            rear_beat = 0;
            break;
        }
        case(PC_AUTOAIM_MODE_FRONT):
        {
            if(input == PC_AUTOAIM_MODE_REAR) 
            {
                *now_autoaim_status = PC_AUTOAIM_MODE_FRONT;
            } 
            else 
            {
                *now_autoaim_status = input;
            }
            rear_beat = 0;
            break;
        }
        case(PC_AUTOAIM_MODE_REAR):
        {
            if(input == PC_AUTOAIM_MODE_IDLE) 
            {
                rear_beat++;
                if(rear_beat >= 100)
                {
                    *now_autoaim_status = PC_AUTOAIM_MODE_IDLE;
                    rear_beat = 0;
                }
                else
                {
                    *now_autoaim_status = PC_AUTOAIM_MODE_REAR;
                }
            } 
            else 
            {
                *now_autoaim_status = input;
                rear_beat = 0;
            }
            break;
        }
    }
}



/**
 * @brief PcComm任务函数
 * 
 */
void PcComm::Task()
{
    for(;;)
    {
        AlivePeriodElapsedCallback();
        UpdataAutoaimData();
        Send_Message();
        osDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief PcComm接收回调函数
 * 
 */
void PcComm::RxCpltCallback()
{
    // 滑动窗口, 判断是否在线
    flag_ += 1;

    DataProcess();
}

/**
 * @brief PcComm数据处理函数
 * 
 */
void PcComm::DataProcess()
{
    if(bsp_usb_rx_buffer[0] == 'S' && bsp_usb_rx_buffer[1] == 'P')
    {
        uint16_t lenth = sizeof(recv_autoaim_data);
        memcpy(&recv_autoaim_data, bsp_usb_rx_buffer, lenth);

        JudgeAutoaimStatus(&recv_autoaim_data.mode, pre_autoaim_status_);

        pre_autoaim_status_ = recv_autoaim_data.mode;
        
        // if(verify_crc16_check_sum(bsp_usb_rx_buffer, lenth))
        // {
        //     memcpy(&recv_autoaim_data, bsp_usb_rx_buffer, lenth);
        // }
    }
    else if(bsp_usb_rx_buffer[0] == recv_navigation_data.start_of_frame)
    {
        memcpy(&recv_navigation_data.linear_x.b, &bsp_usb_rx_buffer[1], 4);
        memcpy(&recv_navigation_data.linear_y.b, &bsp_usb_rx_buffer[5], 4);

        pc_chassis_x_ = (uint16_t)(K_PC * (recv_navigation_data.linear_x.f / 2.1f + C_PC));
        pc_chassis_y_ = (uint16_t)(K_PC * (recv_navigation_data.linear_y.f / 2.1f + C_PC));

        recv_navigation_data.crc16[0] = bsp_usb_rx_buffer[9];
        recv_navigation_data.crc16[1] = bsp_usb_rx_buffer[10];
    }
}
