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
#ifndef PC_COMM_H
#define PC_COMM_H

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
 * @brief 转换联合体
 * 
 */
union PcConv
{
    uint8_t b[4];
    float f;
};

#pragma pack(1)
/**
 * @brief 自瞄发送结构体
 * 
 */
struct PCSendAutoAimData
{
    uint8_t head[2] = {'S','P'};

    uint8_t mode = 0;               // 0-空闲 1-自瞄 2-小能量机关  3-大能量机关

    float q[4];                     // 四元数姿态[w,x,y,z]

    struct
    {
        float yaw_ang;              // yaw轴角度
        float yaw_vel;              // yaw轴角速度
    } yaw;
    
    struct
    {
        float pitch_ang;            // pitch轴角度
        float pitch_vel;            // pitch轴角速度
    } pitch;
    
    struct
    {
        float bullet_speed;         // 子弹速度
        uint16_t bullet_count;      // 子弹累计发送次数
    } bullet;
    
    uint16_t crc16;                 // 校验位
};

/**
 * @brief 自瞄接收结构体
 * 
 */
struct PCRecvAutoAimData
{
    uint8_t head[2] = {'S','P'};
    uint8_t mode = 0;           // 0-空闲 1-自瞄 2-小能量机关  3-大能量机关

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
    
    uint16_t crc16;             // 校验位
};

/**
 * @brief 导航接收结构体
 * 
 */
struct PCRecvNavigationData
{
    uint8_t start_of_frame = 0x6A;
    PcConv linear_x[4] = {0, 0, 0, 0};
    PcConv linear_y[4] = {0, 0, 0, 0};
    uint8_t crc16[2] = {0};
};
#pragma pack()

/**
 * @brief PcComm类
 * 
 */
class PcComm
{
public:
    // 发送自瞄数据
    PCSendAutoAimData send_autoaim_data = 
    {
        {'S','P'},
        0,
        {1,0,0,0},
        {0,0},
        {0,0},
        {0,0},
        0,
    };
    // 接收自瞄数据
    PCRecvAutoAimData recv_autoaim_data = 
    {
        {'S','P'},
        0,
        {0,0,0},
        {0,0,0},
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

    void Init();

    void Task();

    void Send_Message();

    void RxCpltCallback();

    void UpdataAutoaimData();

private:
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif
