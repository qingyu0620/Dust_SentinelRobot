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
#include "bsp_usb.h"
#include "dvc_MCU_comm.h"

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
        .name = "PcComm_task",
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
// void PcComm::UpdataAutoaimData(McuRecvRefereeFastData& recv_fast_data, McuRecvRefereeSlowData& recv_slow_data)
// {
//     float rotation_q[4] = {0};

//     EularAngleToQuaternion(INS.Yaw, INS.Pitch, -INS.Roll, rotation_q);
    
//     memcpy(&send_autoaim_data.q, rotation_q, 16);

//     send_autoaim_data.yaw_angle      = INS.Yaw;
//     send_autoaim_data.pitch_angle    = -INS.Roll;

//     send_autoaim_data.bullet.speed   = recv_fast_data.bullet_speed.f;
//     send_autoaim_data.bullet.count   = recv_fast_data.bullet_number;


//     send_navigation_data.start_of_frame = '@';
    
//     send_navigation_data.stage_enum     = recv_slow_data.stage_enum;
//     send_navigation_data.current_hp     = recv_slow_data.robot_hp;
//     send_navigation_data.middle_buff_status = recv_fast_data.middle_buff_status;
//     send_navigation_data.stage_remain_time = recv_slow_data.stage_remain_time;

//     send_navigation_data.end_of_frame   = '#';
// }

/**
 * @brief PcComm发送信息函数
 * 
 */
void PcComm::SendAutoaimData()
{
    uint16_t length = sizeof(send_autoaim_data);
    uint8_t buffer[length];

    send_autoaim_data.crc16 = 0;

    memcpy(buffer, &send_autoaim_data, length);
    append_crc16_check_sum(buffer, length);

    usb_transmit(buffer, length);
}

/**
 * @brief PcComm发送信息函数
 * 
 */
void PcComm::SendNavigationData()
{
    uint16_t length = sizeof(send_navigation_data);
    uint8_t buffer[length];

    memcpy(buffer, &send_navigation_data, length);

    usb_transmit(buffer, length); 
}

/**
 * @brief PcComm清理自瞄数据函数
 * 
 */
void PcComm::ClearAutoaimData()
{
    recv_autoaim_data.mode = PC_AUTOAIM_MODE_IDLE;

    recv_autoaim_data.yaw_ang = 0;

    recv_autoaim_data.pitch_ang = 0;

    recv_autoaim_data.ratio = 1;
}

/**
 * @brief PcComm清理导航数据函数
 * 
 */
void PcComm::ClearNavigationData()
{
    pc_chassis_x_ = 1024;
    pc_chassis_y_ = 1024;
}

/**
 * @brief PcComm存活周期检测回调函数
 * 
 */
void PcComm::AlivePeriodElapsedCallback()
{
    if(++alive_beat_ >= MAX_PC_DISALIVE_PERIOD)
    {
        if(pre_autoaim_flag_ == autoaim_flag_)
        {
            ClearAutoaimData();
        }
        if(pre_navigation_flag_ == navigation_flag_)
        {
            ClearNavigationData();
        }

        pre_autoaim_flag_ = autoaim_flag_;
        pre_navigation_flag_ = navigation_flag_;

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
    uint16_t send_flag = 0;
    for(;;)
    {
        AlivePeriodElapsedCallback();
        
        send_flag++;

        if(send_flag == 1)
        {
            SendAutoaimData();
        }
        else if(send_flag == 2)
        {
            SendNavigationData();
            send_flag = 0;
        }
        
        osDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief PcComm接收回调函数
 * 
 */
void PcComm::RxCpltCallback()
{
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
        // 滑动窗口, 判断是否在线
        autoaim_flag_ += 1;

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
        // 滑动窗口, 判断是否在线
        navigation_flag_ += 1;

        memcpy(&recv_navigation_data.linear_x.b, &bsp_usb_rx_buffer[1], 4);
        memcpy(&recv_navigation_data.linear_y.b, &bsp_usb_rx_buffer[5], 4);

        recv_navigation_data.all = bsp_usb_rx_buffer[9];

        pc_chassis_x_ = (uint16_t)(K_PC * (recv_navigation_data.linear_x.f / 2.1f + C_PC));
        pc_chassis_y_ = (uint16_t)(K_PC * (recv_navigation_data.linear_y.f / 2.1f + C_PC));

        recv_navigation_data.crc16[0] = bsp_usb_rx_buffer[9];
        recv_navigation_data.crc16[1] = bsp_usb_rx_buffer[10];
    }
}
