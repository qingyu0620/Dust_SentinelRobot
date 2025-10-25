/**
 * @file dvc_MCU_comm.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef MODULES_COMM_DVC_MCU_COMM_H
#define MODULES_COMM_DVC_MCU_COMM_H
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"


enum ChassisSpinMode{
    CHASSIS_SPIN_CLOCKWISE          = 0,
    CHASSIS_SPIN_DISABLE            = 1,
    CHASSIS_SPIN_COUNTER_CLOCK_WISE = 2,
};

struct McuCommData
{
    uint8_t          start_of_frame;     // 帧头
    uint16_t         chassis_speed_x;    // 平移方向：前、后、左、右
    uint16_t         chassis_speed_y;    // 底盘移动总速度
    uint16_t         chassis_rotation;   // 自转：不转、顺时针转、逆时针转
    ChassisSpinMode  chassis_spin;       // 小陀螺：不转、顺时针转、逆时针转
};

struct McuSendData
{
    uint8_t start_of_frame = 0xAB;
    uint8_t armor;
    float yaw;   // 4字节浮点数
    float pitch; // 4字节浮点数
};

struct McuAutoaimData
{
    uint8_t start_of_yaw_frame;
    uint8_t start_of_pitch_frame;
    uint8_t yaw[4];
    float yaw_f;
    uint8_t pitch[4];
    float pitch_f;
};

class McuComm
{
public:

    volatile McuCommData mcu_comm_data_ = {
            0xAB,
            1024,
            1024,
            1024,
            CHASSIS_SPIN_DISABLE,
    };
    McuSendData mcu_send_data_;

    McuAutoaimData mcu_autoaim_data_ = {    0xAC,
                                            0xAD,
                                            {0},
                                            0,
                                            {0},
                                            0,
                                        };
    void Init(CAN_HandleTypeDef *hcan,
              uint8_t can_rx_id,
              uint8_t can_tx_id
              );

    void CanRxCpltCallback(uint8_t *rx_data);

    void CanSendCommand();

    void Task();


protected:
    // 绑定的CAN
    CanManageObject *can_manage_object_;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t can_rx_id_;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t can_tx_id_;
    // 发送缓冲区
    uint8_t tx_data_[8];
    // 内部函数
    void DataProcess();
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

#endif //MODULES_COMM_DVC_MCU_COMM_H
