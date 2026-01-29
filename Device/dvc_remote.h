/**
 * @file dvc_remote_vt02.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __DVC_REMOTE__H__
#define __DVC_REMOTE__H__

/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "alg_math.h"
#include "app_gimbal.h"

/* Exported macros -----------------------------------------------------------*/

#define CLAMP(x, min, max)  ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Remote按键状态枚举
 * 
 */
enum RemoteKeyStatus
{
    REMOTE_KEY_STATUS_FREE = 0,
    REMOTE_KEY_STATUS_PRESS,
};

/**
 * @brief Remote存活状态枚举
 * 
 */
enum RemoteAliveStatus
{
    REMOTE_ALIVE_STATUS_DISABLE = 0,
    REMOTE_ALIVE_STATUS_ENABLE  = 1,
};

/**
 * @brief Remote按键联合体
 * 
 */
union RemoteKeyboard
{
    uint16_t all;
    struct
    {
        uint8_t w : 1;
        uint8_t s : 1;
        uint8_t a : 1;
        uint8_t d : 1;
        uint8_t shift : 1;
        uint8_t ctrl : 1;
        uint8_t q : 1;
        uint8_t e : 1;
        uint8_t r : 1;
        uint8_t f : 1;
        uint8_t g : 1;
        uint8_t z : 1;
        uint8_t x : 1;
        uint8_t c : 1;
        uint8_t v : 1;
        uint8_t b : 1;
    } keycode;
};

/**
 * @brief Remote鼠标左右键枚举
 * 
 */
union RemoteMouseLR
{
    uint8_t all;
    struct 
    {
        uint8_t mouse_l : 1;
        uint8_t mouse_r : 1;
        uint8_t reserved : 6;
    } mousecode;
};

/**
 * @brief Remote鼠标结构体
 * 
 */
struct RemoteMouse
{
    int16_t mouse_x;
    float mouse_y;
    float mouse_z;
    RemoteMouseLR mouse_lr;
};

/**
 * @brief Remote类
 * 
 */
class Remote
{
public:
    // 遥控器状态
    RemoteAliveStatus remote_alive_status = REMOTE_ALIVE_STATUS_DISABLE;

    void Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

    void Task();

    void AlivePeriodElapsedCallback();

    void UartRxCpltCallback(uint8_t* buffer);

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数

protected:
    // uart管理模块
    UartManageObject* uart_manage_object_;

    // 当前时刻flag
    uint32_t flag_ = 0;

    // 前一时刻flag
    uint32_t pre_flag_ = 0;

    virtual void DataProcess(uint8_t* buffer) = 0;

    virtual void ClearData() = 0;

    virtual void Process_Keyboard_Toggle(RemoteKeyboard* current_output, RemoteKeyboard current_raw);
};


/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/







#endif 