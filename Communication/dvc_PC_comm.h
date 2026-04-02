/**
 * @file dvc_PC_comm.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

/* Includes ------------------------------------------------------------------*/

#include "dvc_MCU_comm.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "ins_task.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "crc.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct McuRecvRefereeFastData;      // 前置声明
struct McuRecvRefereeSlowData;      // 前置声明

/**
 * @brief PcComm转换联合体
 * 
 */
union PcConv
{
    uint8_t b[4];
    float f;
};

/**
 * @brief PcComm自瞄模式枚举
 * 
 */
enum PcAutoAimStatus : uint8_t
{
    PC_AUTOAIM_MODE_IDLE = 0,
    PC_AUTOAIM_MODE_REAR,
    PC_AUTOAIM_MODE_FRONT,
};

/**
 * @brief PcComm存活状态枚举
 * 
 */
enum PcAliveState : uint8_t
{
    PC_ALIVE_STATE_ENABLE = 0,
    PC_ALIVE_STATE_DISABLE,
};

/**
 * @brief PcComm状态
 * 
 */
enum PcState : uint8_t 
{
    PC_STATE_ENABLE = 0,
    PC_STATE_DISABLE,
};

#pragma pack(1)

/**
 * @brief PcComm自瞄发送结构体
 * 
 */
struct PCSendAutoAimData
{
    uint8_t head[2] = {'S','P'};

    float q[4];                     // 四元数姿态[w,x,y,z]

    float yaw_angle;                // yaw轴角度

    float pitch_angle;              // pitch轴角度
    
    struct
    {
        float speed;                // 子弹速度
        uint16_t count;             // 子弹累计发送次数
    } bullet;
    
    uint16_t crc16;                 // 校验位
};

/**
 * @brief PcComm导航发送结构体
 * 
 */
struct PCSendNavigationData
{
    uint8_t start_of_frame = '@';

    uint8_t stage_enum = 0;
    uint16_t stage_remain_time = 0;
    uint16_t current_hp = 0;
    uint8_t middle_buff_status = 0;
    
    uint8_t end_of_frame = '#';
};

/**
 * @brief PcComm自瞄接收结构体
 * 
 */
struct PCRecvAutoData
{
    uint8_t head[2] = {'S', 'P'};
    PcAutoAimStatus mode = PC_AUTOAIM_MODE_IDLE;  // 0: 不控制, 1: 后置，2: 前置
    float yaw_ang;
    float pitch_ang;
    PcConv linear_x;
    PcConv linear_y;
    union
    {
        uint8_t all = 0;
        struct 
        {
            uint8_t classis_mode : 1;
            uint8_t scan_status : 1;
            uint8_t fire : 1;
            uint8_t reserved : 5;
        };
    };
    uint16_t crc16;
};

#pragma pack()

/**
 * @brief PcComm类
 * 
 */
class PcComm
{
public:
    // 自瞄发送数据
    PCSendAutoAimData send_autoaim_data = 
    {
        {'S','P'},
        {1,0,0,0},
        0,
        0,
        {0,0},
        0,
    };
    // 导航发送数据
    PCSendNavigationData send_navigation_data = 
    {
        '@',
        0,
        0,
        0,
        0,
        '#',
    };
    // 自瞄接收数据
    PCRecvAutoData recv_auto_data = 
    {
        {'S','P'},
        PC_AUTOAIM_MODE_IDLE,
        0,
        0,
        {0,0,0,0},
        {0,0,0,0},
        0,
        0,
    };

    // 导航x通道值
    uint16_t pc_chassis_x_ = 1024;

    // 导航y通道值
    uint16_t pc_chassis_y_ = 1024;

    void Init();

    void SendAutoaimData();

    void SendNavigationData();

    void RxCpltCallback();

    void UpdataAutoaimData(McuRecvRefereeFastData& recv_fast_data, McuRecvRefereeSlowData& recv_slow_data);

    void JudgeAutoaimStatus(PcAutoAimStatus* now_autoaim_status, PcAutoAimStatus pre_autoaim_status);

    inline PcAliveState GetAlivState() { 
        return pc_alive_state; 
    }

    inline PcAutoAimStatus GetAutoAimStatus() {
        return pre_autoaim_status_;
    }

private:

    uint32_t flag_ = 0;

    uint32_t pre_flag_ = 0;

    uint32_t alive_beat_ = 0;

    PcAliveState pc_alive_state = PC_ALIVE_STATE_DISABLE;
    
    PcAutoAimStatus pre_autoaim_status_ = PC_AUTOAIM_MODE_IDLE;

    uint16_t rear_run_time_ = 0;            // REAR状态运行时间累计
    uint16_t rear_confirm_count_ = 0;       // REAR进入确认计数

    void DataProcess();

    void AlivePeriodElapsedCallback();

    void Task();

    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


