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
#include <algorithm>

/* Private constants ---------------------------------------------------------*/

constexpr float K_PC    = 660.f;
constexpr float C_PC    = 256.f / 165.f;

constexpr uint16_t MAX_PC_DISALIVE_PERIOD       = 200;  // 200ms

// 自瞄状态切换参数
constexpr uint16_t REAR_TO_IDLE_DEBOUNCE_COUNT  = 200;  // 后置退出到IDLE的防抖计数（足够覆盖180°旋转时间）
constexpr uint16_t REAR_ENTRY_CONFIRM_COUNT     = 5;    // 从IDLE进入REAR的连续确认帧数（防止后置误检）

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
        .stack_size = 512,
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
void PcComm::UpdataAutoaimData(McuRecvRefereeFastData& recv_fast_data, McuRecvRefereeSlowData& recv_slow_data)
{
    float rotation_q[4] = {0};

    EularAngleToQuaternion(INS.Yaw, INS.Pitch, -INS.Roll, rotation_q);

    memcpy(&send_autoaim_data.q, rotation_q, 16);

    send_autoaim_data.yaw_angle      = INS.Yaw;
    send_autoaim_data.pitch_angle    = -INS.Roll;

    send_autoaim_data.bullet.speed   = recv_fast_data.bullet_speed.f;
    send_autoaim_data.bullet.count   = recv_fast_data.bullet_number;

    send_navigation_data.start_of_frame = '@';

    send_navigation_data.stage_enum     = recv_slow_data.stage_enum;
    send_navigation_data.current_hp     = recv_slow_data.robot_hp;
    send_navigation_data.middle_buff_status = recv_fast_data.middle_buff_status;
    send_navigation_data.stage_remain_time = recv_slow_data.stage_remain_time;

    send_navigation_data.end_of_frame   = '#';
}

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
 * @brief PcComm存活周期检测回调函数
 * 
 */
void PcComm::AlivePeriodElapsedCallback()
{
    if(++alive_beat_ >= MAX_PC_DISALIVE_PERIOD)
    {
        if(flag_ == pre_flag_) {
            pc_alive_state = PC_ALIVE_STATE_DISABLE;
        } else {
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
    PcAutoAimStatus input = *now_autoaim_status;

    switch (pre_autoaim_status)
    {
        case PC_AUTOAIM_MODE_IDLE:
        {
            if (input == PC_AUTOAIM_MODE_FRONT)
            {
                // 前置发现目标，立即进入FRONT
                *now_autoaim_status = PC_AUTOAIM_MODE_FRONT;
                rear_confirm_count_ = 0;
            }
            else if (input == PC_AUTOAIM_MODE_REAR)
            {
                // 后置报目标，需连续确认多帧才进入REAR（防止误检触发180°旋转）
                rear_confirm_count_++;
                if (rear_confirm_count_ >= REAR_ENTRY_CONFIRM_COUNT) {
                    *now_autoaim_status = PC_AUTOAIM_MODE_REAR;
                    rear_run_time_ = 0;
                    rear_confirm_count_ = 0;
                } else {
                    *now_autoaim_status = PC_AUTOAIM_MODE_IDLE;
                }
            }
            else
            {
                // 无目标，保持IDLE
                *now_autoaim_status = PC_AUTOAIM_MODE_IDLE;
                rear_confirm_count_ = 0;
            }

            break;
        }
        case PC_AUTOAIM_MODE_FRONT:
        {
            if (input == PC_AUTOAIM_MODE_FRONT)
            {
                // 前置持续识别目标，保持FRONT
                *now_autoaim_status = PC_AUTOAIM_MODE_FRONT;
            }
            else if (input == PC_AUTOAIM_MODE_IDLE)
            {
                // 前置无目标，立即退出到IDLE（避免自瞄数据导致云台疯转）
                *now_autoaim_status = PC_AUTOAIM_MODE_IDLE;
            }
            else
            {   // input == PC_AUTOAIM_MODE_REAR
                // 前置丢目标且后置发现目标，立即切到REAR
                *now_autoaim_status = PC_AUTOAIM_MODE_REAR;
                rear_run_time_ = 0;
            }

            break;
        }
        case PC_AUTOAIM_MODE_REAR:
        {
            // 饱和计数，防止uint16溢出
            if (rear_run_time_ < REAR_TO_IDLE_DEBOUNCE_COUNT + 1) {
                rear_run_time_++;
            }

            if (input == PC_AUTOAIM_MODE_FRONT)
            {
                // 前置发现敌人，立即切换（前置优先）
                *now_autoaim_status = PC_AUTOAIM_MODE_FRONT;
                rear_run_time_ = 0;
            }
            else if (input == PC_AUTOAIM_MODE_IDLE)
            {
                // 后置无目标，判断是否已度过最小保护时间
                if (rear_run_time_ >= REAR_TO_IDLE_DEBOUNCE_COUNT) {
                    *now_autoaim_status = PC_AUTOAIM_MODE_IDLE;
                    rear_run_time_ = 0;
                } else {
                    // 还在保护时间内，继续保持REAR（云台正在旋转）
                    *now_autoaim_status = PC_AUTOAIM_MODE_REAR;
                }
            }
            else
            {   // input == PC_AUTOAIM_MODE_REAR
                // 后置持续看到目标，继续旋转搜索
                *now_autoaim_status = PC_AUTOAIM_MODE_REAR;
            }

            break;
        }
        default:
        {
            // 异常状态，安全回退到IDLE
            *now_autoaim_status = PC_AUTOAIM_MODE_IDLE;
            rear_run_time_ = 0;
            rear_confirm_count_ = 0;
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

    for (;;)
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
        pre_flag_ += 1;

        uint16_t lenth = sizeof(recv_auto_data);
        memcpy(&recv_auto_data, bsp_usb_rx_buffer, lenth);

        // 使用临时变量保存处理后的状态，不修改原始接收数据
        PcAutoAimStatus actual_status = recv_auto_data.mode;
        JudgeAutoaimStatus(&actual_status, pre_autoaim_status_);

        pre_autoaim_status_ = actual_status;

        pc_chassis_x_ = (uint16_t)(K_PC * (recv_auto_data.linear_x.f / 2.1f + C_PC));
        pc_chassis_y_ = (uint16_t)(K_PC * (recv_auto_data.linear_y.f / 2.1f + C_PC));

        pc_chassis_x_ = std::clamp(pc_chassis_x_, static_cast<uint16_t>(364), static_cast<uint16_t>(1684));
        pc_chassis_y_ = std::clamp(pc_chassis_y_, static_cast<uint16_t>(364), static_cast<uint16_t>(1684));
        
        // if(verify_crc16_check_sum(bsp_usb_rx_buffer, lenth))
        // {
        //     memcpy(&recv_autoaim_data, bsp_usb_rx_buffer, lenth);
        // }
    }
}
