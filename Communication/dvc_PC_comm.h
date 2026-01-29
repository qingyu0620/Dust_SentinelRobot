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
#ifndef __DVC_PC_COMM_H__
#define __DVC_PC_COMM_H__

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "ins_task.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "crc.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

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
enum PcAliveState
{
    PC_ALIVE_STATE_ENABLE = 0,
    PC_ALIVE_STATE_DISABLE,
};

#pragma pack(1)

/**
 * @brief PcComm自瞄发送结构体
 * 
 */
struct PCSendAutoAimData1
{
    uint8_t head[2] = {'S','P'};

    PcAutoAimStatus mode;               // 0-空闲 1-自瞄不开火 2-自瞄开火

    float q[4];                     // 四元数姿态[w,x,y,z]

    struct
    {
        float ang;                  // yaw轴角度
        float vel;                  // yaw轴角速度
    } yaw;
    
    struct
    {
        float ang;                  // pitch轴角度
        float vel;                  // pitch轴角速度
    } pitch;
    
    struct
    {
        float speed;                // 子弹速度
        uint16_t count;             // 子弹累计发送次数
    } bullet;
    
    uint16_t crc16;                 // 校验位
};

/**
 * @brief PcComm自瞄发送结构体
 * 
 */
struct PCSendAutoAimData2
{
    uint8_t start_of_frame = 0xA6;

    uint16_t current_hp;

    uint16_t checksum = 0;
};

/**
 * @brief PcComm自瞄接收结构体
 * 
 */
struct PCRecvAutoAimData
{
    uint8_t head[2] = {'S','P'};

    PcAutoAimStatus mode = PC_AUTOAIM_MODE_IDLE;           // 0-空闲 1-后置摄像头 2-前置摄像头

    struct
    {
        float yaw_ang;          // yaw轴角度
        float yaw_vel;          // yaw轴角速度
        float yaw_acc;          // yaw轴角加速度
    } yaw;
    
    struct
    {
        float pitch_ang;        // pitch轴角度
        float pitch_vel;        // pitch轴角速度
        float pitch_acc;        // pitch轴角加速度
    } pitch;
    
    uint8_t flag;

    uint16_t crc16;             // 校验位
};



/**
 * @brief PcComm导航接收结构体
 * 
 */
struct PCRecvNavigationData
{
    uint8_t start_of_frame = 0x6A;

    PcConv linear_x;
    PcConv linear_y;

    uint8_t crc16[2] = {0, 0};
};

#pragma pack()

/**
 * @brief PcComm类
 * 
 */
class PcComm
{
public:
    // 发送自瞄数据1
    PCSendAutoAimData1 send_autoaim_data1 = 
    {
        {'S','P'},
        PC_AUTOAIM_MODE_IDLE,
        {1,0,0,0},
        {0,0},
        {0,0},
        {0,0},
        0,
    };
    // 发送自瞄数据2
    PCSendAutoAimData2 send_autoaim_data2 = 
    {
        0xA6,
        0,
        0,
    };
    // 接收自瞄数据
    PCRecvAutoAimData recv_autoaim_data = 
    {
        {'S','P'},
        PC_AUTOAIM_MODE_IDLE,
        {0,0,0},
        {0,0,0},
        0,
        0,
    };
    // 接收导航数据
    PCRecvNavigationData recv_navigation_data = 
    {
        0x6A,
        {0,0,0,0},
        {0,0,0,0},
        {0,0},
    };

    // 导航x通道值
    uint16_t pc_chassis_x_ = 1024;

    // 导航y通道值
    uint16_t pc_chassis_y_ = 1024;

    void Init();

    void Task();

    void Send_Message();

    void RxCpltCallback();

    void UpdataAutoaimData();

    void JudgeAutoaimStatus(PcAutoAimStatus* now_autoaim_status, PcAutoAimStatus pre_autoaim_status);

private:

    uint32_t flag_ = 0;

    uint32_t pre_flag_ = 0;

    uint32_t alive_beat_ = 0;

    PcAliveState pc_alive_state = PC_ALIVE_STATE_DISABLE;
    
    PcAutoAimStatus pre_autoaim_status_ = PC_AUTOAIM_MODE_IDLE;

    void ClearData();

    void DataProcess();

    void AlivePeriodElapsedCallback();

    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif
