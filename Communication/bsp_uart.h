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

/* Exported macros -----------------------------------------------------------*/

// 串口缓冲区大小
#define UART_BUFFER_LENGTH      128

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ：UART回调函数定义
 * 
 */
typedef void (* Uart_Callback)(uint8_t* buffer, uint16_t length);


/**
 * @brief ：UART处理结构体
 * 
 */
typedef struct
{
    UART_HandleTypeDef* uart_handle;
    uint8_t tx_buffer[UART_BUFFER_LENGTH];
    uint8_t rx_buffer[UART_BUFFER_LENGTH];
    uint16_t rx_buffer_length;
    Uart_Callback callback_function;
} Uart_Instance;

/* Exported variables --------------------------------------------------------*/

extern Uart_Instance uart1_instance;
extern Uart_Instance uart6_instance;

/* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

// 函数声明
void uart_init(UART_HandleTypeDef* huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

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