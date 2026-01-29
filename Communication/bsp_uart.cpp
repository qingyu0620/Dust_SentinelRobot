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

__weak void UartHardFault()
{
	while(1);
}

/**
 * @brief ：printf重定向函数（不用管）
 * 
 * @param ch 
 * @return int 
 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
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
	if(uart_manage_object != NULL)
	{
		uart_manage_object->uart_handle = huart;
		uart_manage_object->callback_function = callback_function;
		uart_manage_object->rx_buffer_length = rx_buffer_length;
		HAL_UARTEx_ReceiveToIdle_DMA(uart_manage_object->uart_handle, uart_manage_object->rx_buffer, uart_manage_object->rx_buffer_length);
	}
	else
	{
		UartHardFault();
	}
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
	if(uart_manage_object != NULL)
	{
		HAL_UART_AbortReceive(uart_manage_object->uart_handle);
		memset(uart_manage_object->rx_buffer, 0, uart_manage_object->rx_buffer_length);
		HAL_UARTEx_ReceiveToIdle_DMA(uart_manage_object->uart_handle, uart_manage_object->rx_buffer, uart_manage_object->rx_buffer_length);
	}
	else
	{
		UartHardFault();
	}
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
	if(uart_manage_object->callback_function != NULL)
	{
		uart_manage_object->callback_function(uart_manage_object->rx_buffer, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(uart_manage_object->uart_handle, uart_manage_object->rx_buffer, uart_manage_object->rx_buffer_length);
	}
	else
	{
		UartHardFault();
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

