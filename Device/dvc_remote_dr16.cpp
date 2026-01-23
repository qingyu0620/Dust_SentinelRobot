/**
 * @file dvc_remote_dji.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote_dr16.h"
#include "app_gimbal.h"

/* Private macros ------------------------------------------------------------*/

#define K_NORM      1.f / 660.f
#define C_NORM      -256.f / 165.f
#define K_PITCH     13.f / 13200.f
#define C_PITCH     -832.f / 825.f

#define CLAMP(x, min, max)  ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief DjiDR16遥控初始化函数
 * 
 * @param huart uart句柄
 * @param callback_function 回调函数
 * @param rx_buffer_length 接受缓冲区长度
 */
void RemoteDjiDR16::Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
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
    osThreadNew(RemoteDjiDR16::TaskEntry, this, &kRemoteTaskAttr);
}

/**
 * @brief DjiDR16遥控
 * 
 * @param argument 
 */
void RemoteDjiDR16::TaskEntry(void *argument)
{
    RemoteDjiDR16 *self = static_cast<RemoteDjiDR16 *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
};

/**
 * @brief DjiDR16清理数据函数
 * 
 */
void RemoteDjiDR16::ClearData()
{
    output_.remote.pitch = K_PITCH * 1024 + C_PITCH;
    output_.remote.chassis_x = output_.remote.chassis_y = output_.remote.rotation = 1024;
    output_.remote.switch_l = output_.remote.switch_r = 3;
}

/**
 * @brief DjiDR16检测在线回调函数
 * 
 */
void RemoteDjiDR16::AlivePeriodElapsedCallback()
{
    // 判断时间段内是否掉线
    if(pre_flag_ == flag_)
    {
        // 断开连接
        remote_dji_alive_status = REMOTE_DJI_STATUS_DISABLE;
        ClearData();
    }
    else
    {
        remote_dji_alive_status = REMOTE_DJI_STATUS_ENABLE;
    }
    pre_flag_ = flag_;
}

/**
 * @brief 
 * 
 * @param current_raw 
 */
void RemoteDjiDR16::Process_Keyboard_Toggle(RemoteDR16Keyboard current_raw)
{
    static uint16_t last_raw_all = 0;
    static RemoteDR16Keyboard toggle_output = {0};

    uint16_t trigger = current_raw.all & (~last_raw_all);

    uint16_t toggle_mask = 0xFFC0; 

    toggle_output.all ^= (trigger & toggle_mask);
    
    uint16_t normal_mask = ~toggle_mask;
    output_.keyboard.all = (toggle_output.all & toggle_mask) | (current_raw.all & normal_mask);

    last_raw_all = current_raw.all;
}

/**
 * @brief DjiDR16任务函数
 * 
 */
void RemoteDjiDR16::Task()
{
    for(;;)
    {
        AlivePeriodElapsedCallback();
        osDelay(pdMS_TO_TICKS(50));     // 请勿修改频率
    }
}

/**
 * @brief DjiDR16转换函数
 * 
 * @param buffer 传入遥控数据
 */
void RemoteDjiDR16::UartRxCpltCallback(uint8_t* buffer)
{
    // 滑动窗口, 判断是否在线
    flag_ += 1;
    
    DataProcess(buffer);
}

/**
 * @brief DjiDR16数据处理函数
 * 
 */
void RemoteDjiDR16::DataProcess(uint8_t* buffer)
{
    // /****************************   原始数据    ****************************/


    // raw_data_.rc.ch0 =  ((int16_t)buffer[0]        | ((int16_t)buffer[1] << 8)) & 0x07FF;
    // raw_data_.rc.ch1 = (((int16_t)buffer[1] >> 3)  | ((int16_t)buffer[2] << 5)) & 0x07FF;
    // raw_data_.rc.ch2 = (((int16_t)buffer[2] >> 6)  | ((int16_t)buffer[3] << 2)  | ((int16_t)buffer[4] << 10)) &  0x07FF;
    // raw_data_.rc.ch3 = (((int16_t)buffer[4] >> 1)  | ((int16_t)buffer[5] << 7)) & 0x07FF;

    // raw_data_.rc.s1 = ((buffer[5] >> 4) & 0x000C) >> 2;
    // raw_data_.rc.s2 = ((buffer[5] >> 4) & 0x0003);

    // int16_t dx = ((int16_t)buffer[6]) | ((int16_t)buffer[7] << 8);
    // int16_t dy = ((int16_t)buffer[8]) | ((int16_t)buffer[9] << 8);

    // raw_data_.mouse.x = CLAMP(dx * 30, INT16_MIN, INT16_MAX);
    // raw_data_.mouse.y = CLAMP(dy * 2, INT16_MIN, INT16_MAX);

    // raw_data_.mouse.pl = buffer[12];
    // raw_data_.mouse.pr = buffer[13];

    // raw_data_.keyboard.all = (int16_t)buffer[14];


    // /****************************   遥控数据    ****************************/

    // // 上板数据
    // output_.remote.pitch = K_PITCH * raw_data_.rc.ch3 + C_PITCH;
    // output_.remote.pitch = CLAMP(output_.remote.pitch, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

    // // 下板数据
    // output_.remote.chassis_x  = raw_data_.rc.ch1;
    // output_.remote.chassis_y  = raw_data_.rc.ch0;
    // output_.remote.rotation   = raw_data_.rc.ch2;

    // // 通用数据
    // output_.remote.switch_l = raw_data_.rc.s1;
    // output_.remote.switch_r = raw_data_.rc.s2;


    // /****************************   键鼠数据    ****************************/

    // // 鼠标数据
    // output_.mouse.mouse_x = (1684 + 1320 * ((int16_t)raw_data_.mouse.x - 32767) / 65535);
    // output_.mouse.mouse_y += (float)raw_data_.mouse.y / 32767.f;
    // output_.mouse.mouse_y = CLAMP(output_.mouse.mouse_y, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

    // output_.mouse.press_l = raw_data_.mouse.pl;
    // output_.mouse.press_r = raw_data_.mouse.pr;

    // // 键盘数据
    // Process_Keyboard_Toggle(raw_data_.keyboard);
}
