/**
 * @file bsp_usart.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-08
 * 
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __BSP_UART_H__
#define __BSP_UART_H__

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

/* Exported macros -----------------------------------------------------------*/

// 串口缓冲区大小
#define UART_BUFFER_LENGTH      128

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART回调函数定义
 * 
 */
typedef void (* Uart_Callback)(uint8_t* buffer, uint16_t length);

/**
 * @brief UART错误码
 * 
 * @note 设计成联合体是为了方便清0
 */
union UartErrorCode
{
    uint8_t all;
    struct 
    {
        uint8_t overrun_error : 1;      // 缓冲区溢出
        uint8_t frame_error : 1;        // 帧错误
        uint8_t noise_error : 1;        // 噪声错误
        uint8_t parity_error : 1;       // 奇偶校验位错误
        uint8_t dma_error : 1;          // DMA传输错误
        uint8_t reseverd : 3;           // 保留位
    } check;
};

/**
 * @brief UART管理模块结构体
 * 
 */
struct UartManageObject
{
    UART_HandleTypeDef* uart_handle;
    UartErrorCode error_code = {0};
    uint8_t tx_buffer[UART_BUFFER_LENGTH];
    uint8_t rx_buffer[UART_BUFFER_LENGTH];
    uint16_t rx_buffer_length;
    Uart_Callback callback_function;
};

/* Exported variables --------------------------------------------------------*/

extern UartManageObject uart1_manage_object;
extern UartManageObject uart2_manage_object;
extern UartManageObject uart3_manage_object;
extern UartManageObject uart4_manage_object;
extern UartManageObject uart5_manage_object;
extern UartManageObject uart6_manage_object;

/* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

// 函数声明
void uart_init(UART_HandleTypeDef* huart, Uart_Callback callback_function, uint16_t rx_buffer_length);
void uart_reinit(UART_HandleTypeDef* huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

// 回调函数声明
void uart1_callback_function(uint8_t* buffer, uint16_t length);
void uart3_callback_function(uint8_t* buffer, uint16_t length);
void uart6_callback_function(uint8_t* buffer, uint16_t length);

// printf重定义函数声明（当文件为cpp时需添加）
int __io_putchar(int ch);
int _write(int file, char *ptr, int len);

#ifdef __cplusplus
}
#endif

#endif