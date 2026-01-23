/**
 * @file dvc_remote_dji.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __DVC_REMOTE_DJI_H__
#define __DVC_REMOTE_DJI_H__

/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief DjiDR16存活状态
 * 
 */
enum RemoteDR16AliveStatus
{
    REMOTE_DJI_STATUS_DISABLE = 0,
    REMOTE_DJI_STATUS_ENABLE  = 1,
};

/**
 * @brief DjiDR16按键状态
 * 
 */
enum RemoteDR16SwitchStatus
{
    SWITCH_UP    = (uint8_t)1,
    SWITCH_MID   = (uint8_t)3,
    SWITCH_DOWN  = (uint8_t)2,
};

enum RemoteDR16KeyStatus
{
    REMOTE_DR16_KEY_STATUS_FREE = 0,
    REMOTE_DR16_KEY_STATUS_PRESS,
};

/**
 * @brief DjiDR16键盘联合体
 * 
 */
union RemoteDR16Keyboard
{
    uint16_t all;
    struct
    {
        uint16_t w : 1, 
                 s : 1,
                 a : 1,
                 d : 1,
                 shift : 1, 
                 ctrl : 1,
                 q : 1, 
                 e : 1;
        uint16_t reserved : 8;
    } keycode;
};

/**
 * @brief 
 * 
 */
union RemoteDR16MouseLR
{
    uint8_t all;
    struct 
    {
        uint8_t mouse_l : 2;
        uint8_t mouse_r : 2;
        uint8_t reserved : 4;
    } mousecode;
};

/**
 * @brief DjiDR16原始数据
 * 
 */
struct RemoteDR16RawData
{
    struct 
    {
        uint16_t ch0, ch1, ch2, ch3;
        uint8_t s1, s2;
    } rc;

    struct
    {
        int32_t x, y, z;
        uint8_t pl, pr;
    } mouse;

    RemoteDR16Keyboard keyboard;
};

/**
 * @brief DjiDR16输出
 * 
 */
struct RemoteDR16OutputData
{
    struct 
    {
        uint8_t switch_l, switch_r;
        float chassis_x, chassis_y;      // x, y, r 采用右手系
        float rotation;
        float pitch;
    } remote;                            // 遥控数据

    struct
    {
        uint16_t mouse_x;
        float mouse_y;
        uint8_t press_l, press_r;
    } mouse;                             // 鼠标数据

    RemoteDR16Keyboard keyboard;         // 键盘数据
};

/**
 * @brief DjiDR16遥控器
 * 
 */
class RemoteDjiDR16
{
public:
    // 遥控器输出数据
    RemoteDR16OutputData output_;

    // 遥控器状态
    RemoteDR16AliveStatus remote_dji_alive_status = REMOTE_DJI_STATUS_DISABLE;

    void Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

    void Task();

    void AlivePeriodElapsedCallback();

    void UartRxCpltCallback(uint8_t* buffer);

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数

protected:
    // uart管理模块
    UartManageObject* uart_manage_object_;

    // 原始数据
    RemoteDR16RawData raw_data_;

    // 当前时刻flag
    uint32_t flag_ = 0;

    // 前一时刻flag
    uint32_t pre_flag_ = 0;

    void ClearData();

    void Process_Keyboard_Toggle(RemoteDR16Keyboard current_raw);

    void DataProcess(uint8_t* buffer);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif