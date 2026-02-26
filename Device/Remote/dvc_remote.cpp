/**
 * @file dvc_remote.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-27
 * 
 * @copyright Copyright (c) 2026
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote.h"

/* Private macros ------------------------------------------------------------*/

#define KEYBOARD_MODE   0xFFC0      // 从高到低对应 1为toggle键 0为normal键

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Remote初始化函数
 * 
 * @param huart 句柄
 * @param callback_function Uart回调函数
 * @param rx_buffer_length Rx缓冲区长度
 */
void Remote::Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
{
    uart_manage_object_->uart_handle = huart;
    uart_manage_object_->callback_function = callback_function;
    uart_manage_object_->rx_buffer_length = rx_buffer_length;
    uart_init(huart, callback_function, rx_buffer_length);

    static const osThreadAttr_t kRemoteTaskAttr = {
        .name = "kRemoteTaskAttr",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Remote::TaskEntry, this, &kRemoteTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Remote::TaskEntry(void *argument)
{
    Remote *self = static_cast<Remote *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
};

/**
 * @brief Remote掉线周期检测函数
 * 
 */
void Remote::AlivePeriodElapsedCallback()
{
    // 判断时间段内是否掉线
    if(pre_flag_ == flag_)
    {
        // 断开连接
        remote_alive_status = REMOTE_ALIVE_STATUS_DISABLE;
        ClearData();
    }
    else
    {
        remote_alive_status = REMOTE_ALIVE_STATUS_ENABLE;
    }

    pre_flag_ = flag_;
}

/**
 * @brief Remote按键处理函数
 * 
 * @param current_output 输出数据
 * @param current_raw 原始数据
 */
void Remote::Process_Keyboard_Toggle(RemoteKeyboard* current_output, RemoteKeyboard current_raw)
{
    static uint16_t last_raw_all = 0;
    static RemoteKeyboard toggle_output = {0};

    uint16_t trigger = current_raw.all & (~last_raw_all);

    uint16_t toggle_mask = KEYBOARD_MODE; 

    toggle_output.all ^= (trigger & toggle_mask);
    
    uint16_t normal_mask = ~toggle_mask;
    current_output->all = (toggle_output.all & toggle_mask) | (current_raw.all & normal_mask);

    last_raw_all = current_raw.all;
}

/**
 * @brief Remote任务函数
 * 
 */
void Remote::Task()
{
    for(;;)
    {
        AlivePeriodElapsedCallback();
        osDelay(pdMS_TO_TICKS(50));     // 请勿修改频率
    }
}

/**
 * @brief RemoteUart接收回调函数
 * 
 * @param buffer 
 */
void Remote::UartRxCpltCallback(uint8_t* buffer)
{
    // 滑动窗口, 判断是否在线
    flag_ += 1;
    
    DataProcess(buffer);
}
