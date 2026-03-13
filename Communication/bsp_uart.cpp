/**
 * @file bsp_uart.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-27
 * 
 * @copyright Copyright (c) 2026
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private types -------------------------------------------------------------*/

typedef struct UartMapEntry
{
	USART_TypeDef* Instance;
	UartManageObject* manage_obj;
} UartMapEntry;

/* Private variables ---------------------------------------------------------*/

UartManageObject uart1_manage_object = {0};
UartManageObject uart2_manage_object = {0};
UartManageObject uart3_manage_object = {0};
UartManageObject uart4_manage_object = {0};
UartManageObject uart5_manage_object = {0};
UartManageObject uart6_manage_object = {0};

static const UartMapEntry uart_map_inquiry_[]
{
	{USART1, &uart1_manage_object},
	{USART2, &uart2_manage_object},
	{USART3, &uart3_manage_object},
	{UART4,  &uart4_manage_object},
	{UART5,  &uart5_manage_object},
	{USART6, &uart6_manage_object},
};

/* Private macros ------------------------------------------------------------*/

#define UART_MAP_SIZE	(sizeof(uart_map_inquiry_)) / sizeof(UartMapEntry)

/* Private function declarations ---------------------------------------------*/

/**
 * @brief printf重定向函数（不用管）
 * 
 * @param file 
 * @param ptr 
 * @param len 
 * @return int 
 */
int _write(int file, char *ptr, int len)
{
    UartManageObject* uart_obj = &uart6_manage_object;
    
    // 使用 ISR 安全版本的临界区，兼容中断和任务上下文
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    
    for (int i = 0; i < len; i++) 
	{
        uart_obj->tx_buffer[uart_obj->tx_head] = ptr[i];
        uart_obj->tx_head = (uart_obj->tx_head + 1) % UART_BUFFER_LENGTH;
    }
    
    if (!uart_obj->tx_busy) 
	{
        uart_obj->tx_busy = 1;
        
        // 计算要发送的数据长度
        uint16_t send_len;
        if (uart_obj->tx_head >= uart_obj->tx_tail) {
            send_len = uart_obj->tx_head - uart_obj->tx_tail;
        } else {
            // 环形缓冲区回绕时，只发送到缓冲区末尾
            send_len = UART_BUFFER_LENGTH - uart_obj->tx_tail;
        }

        if (send_len > 0) {
			uart_obj->tx_sending_len = send_len;
            HAL_UART_Transmit_DMA(uart_obj->uart_handle, &uart_obj->tx_buffer[uart_obj->tx_tail], send_len);
        } else {
            uart_obj->tx_busy = 0;  // 没有数据可发送，清除忙标志
        }
    }
    
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    
    return len;
}

/* function prototypes -------------------------------------------------------*/

/**
 * @brief Uart管理模块获取函数
 * 
 * @param huart 句柄
 * @return UartManageObject* Uart管理模块指针
 */
static UartManageObject* GetUartManageObject(UART_HandleTypeDef* huart)
{
	for(uint8_t i = 0; i < UART_MAP_SIZE; i++) 
	{
        if(huart->Instance == uart_map_inquiry_[i].Instance) 
		{
            return uart_map_inquiry_[i].manage_obj;
        }
    }
    return NULL;
}

/**
 * @brief Uart初始化函数
 * 
 * @param huart 句柄
 * @param callback_function 回调函数
 * @param rx_buffer_length 接收缓冲区长度
 */
void uart_init(UART_HandleTypeDef* huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
{
	UartManageObject* uart_manage_object = GetUartManageObject(huart);
	if(!uart_manage_object) {
		configASSERT(false);
	}

	uart_manage_object->uart_handle = huart;
	uart_manage_object->callback_function = callback_function;
	uart_manage_object->rx_buffer_length = rx_buffer_length;
	HAL_UARTEx_ReceiveToIdle_DMA(uart_manage_object->uart_handle, uart_manage_object->rx_buffer, uart_manage_object->rx_buffer_length);
}
	
/**
 * @brief Uart再初始化
 * 
 * @param huart 句柄
 * @param callback_function 回调函数
 * @param rx_buffer_length 接收缓冲区长度
 */
void uart_reinit(UART_HandleTypeDef* huart, Uart_Callback callback_function, uint16_t rx_buffer_length)
{
	UartManageObject* uart_manage_object = GetUartManageObject(huart);
	if(!uart_manage_object) {
		configASSERT(false);
	}

	HAL_UART_AbortReceive(uart_manage_object->uart_handle);
	memset(uart_manage_object->rx_buffer, 0, uart_manage_object->rx_buffer_length);
	HAL_UARTEx_ReceiveToIdle_DMA(uart_manage_object->uart_handle, uart_manage_object->rx_buffer, uart_manage_object->rx_buffer_length);
}

/** 
 * @brief Uart接收DMA空闲中断函数
 * 
 * @param huart 句柄
 * @param Size 接收长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	UartManageObject* uart_manage_object = GetUartManageObject(huart);

	uart_manage_object->callback_function(uart_manage_object->rx_buffer, Size);
	memset(uart_manage_object->rx_buffer, 0, uart_manage_object->rx_buffer_length);
	HAL_UARTEx_ReceiveToIdle_DMA(uart_manage_object->uart_handle, uart_manage_object->rx_buffer, uart_manage_object->rx_buffer_length);
}

/**
 * @brief Uart发送完成回调函数
 * 
 * @param huart 
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UartManageObject* uart_obj = GetUartManageObject(huart);

	uart_obj->tx_tail = (uart_obj->tx_tail + uart_obj->tx_sending_len) % UART_BUFFER_LENGTH;
	uart_obj->tx_sending_len = 0;
    
	if (uart_obj->tx_head != uart_obj->tx_tail) {
        uint16_t send_len = (uart_obj->tx_head >= uart_obj->tx_tail) ? 
                            (uart_obj->tx_head - uart_obj->tx_tail) : 
                            (UART_BUFFER_LENGTH - uart_obj->tx_tail + uart_obj->tx_head);
        if (send_len > 0) {
			uart_obj->tx_sending_len = send_len;
            HAL_UART_Transmit_DMA(huart, &uart_obj->tx_buffer[uart_obj->tx_tail], send_len);
        }
    } 
	else {
        uart_obj->tx_busy = 0;
    }
}

/**
 * @brief Uart错误帧回调函数
 * 
 * @param huart 
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uint32_t error_code = huart->ErrorCode;
	UartManageObject* uart_manage_object = GetUartManageObject(huart);

	HAL_UART_DMAStop(uart_manage_object->uart_handle);

	if (error_code & HAL_UART_ERROR_ORE) 
	{
		uart_manage_object->error_code.check.overrun_error = 1;
		__HAL_UART_CLEAR_OREFLAG(huart);
	}
	if (error_code & HAL_UART_ERROR_FE)
	{
		uart_manage_object->error_code.check.frame_error = 1;
		__HAL_UART_CLEAR_FEFLAG(huart);
	}
	if (error_code & HAL_UART_ERROR_NE)
	{
		uart_manage_object->error_code.check.noise_error = 1;
		__HAL_UART_CLEAR_NEFLAG(huart);
	}
	if (error_code & HAL_UART_ERROR_PE)
	{
		uart_manage_object->error_code.check.parity_error = 1;
		__HAL_UART_CLEAR_PEFLAG(huart);
	}
	if (error_code & HAL_UART_ERROR_DMA)
	{
		uart_manage_object->error_code.check.dma_error = 1;
	}

	memset(uart_manage_object->rx_buffer, 0, UART_BUFFER_LENGTH);

	HAL_UARTEx_ReceiveToIdle_DMA(uart_manage_object->uart_handle, uart_manage_object->rx_buffer, uart_manage_object->rx_buffer_length);

}

