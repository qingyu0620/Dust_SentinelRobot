/**
 * @file alg_math.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "alg_math.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 布尔值反转
 *
 * @param Value 布尔值地址
 */
void math_boolean_logical_not(bool *value)
{
    if (*value == false)
    {
        *value = true;
    }
    else if (*value == true)
    {
        *value = false;
    }
}

/**
 * @brief 16位大小端转换
 *
 * @param Address 地址
 */
void math_endian_reverse_16(void *address)
{
    uint8_t *temp_address_8 = (uint8_t *) address;
    uint16_t *temp_address_16 = (uint16_t *) address;
    *temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];
}

/**
 * @brief 16位大小端转换
 *
 * @param source 源数据地址
 * @param destination 目标存储地址
 * @return uint16_t 结果
 */
uint16_t math_endian_reverse_16(void *source, void *destination)
{
    uint8_t *temp_address_8 = (uint8_t *) source;
    uint16_t temp_address_16;
    temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];

    if (destination != nullptr)
    {
        uint8_t *temp_source, *temp_destination;
        temp_source = (uint8_t *) source;
        temp_destination = (uint8_t *) destination;

        temp_destination[0] = temp_source[1];
        temp_destination[1] = temp_source[0];
    }

    return temp_address_16;
}

/**
 * @brief 32位大小端转换
 *
 * @param address 地址
 */
void math_endian_reverse_32(void *address)
{
    uint8_t *temp_address_8 = (uint8_t *) address;
    uint32_t *temp_address_32 = (uint32_t *) address;
    *temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];
}

/**
 * @brief 32位大小端转换
 *
 * @param Source 源数据地址
 * @param Destination 目标存储地址
 * @return uint32_t 结果
 */
uint32_t math_endian_reverse_32(void *source, void *destination)
{
    uint8_t *temp_address_8 = (uint8_t *) source;
    uint32_t temp_address_32;
    temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];

    if (destination != nullptr)
    {
        uint8_t *temp_source, *temp_destination;
        temp_source = (uint8_t *) source;
        temp_destination = (uint8_t *) destination;

        temp_destination[0] = temp_source[3];
        temp_destination[1] = temp_source[2];
        temp_destination[2] = temp_source[1];
        temp_destination[3] = temp_source[0];
    }

    return temp_address_32;
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param aength 被加的数据的数量, 注意不是字节数
 * @return uint8_t 结果
 */
uint8_t math_sum_8(uint8_t *address, uint32_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint16_t 结果
 */
uint16_t math_sum_16(uint16_t *address, uint32_t length)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint32_t 结果
 */
uint32_t math_sum_32(uint32_t *address, uint32_t length)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief sinc函数的实现
 *
 * @param x 输入
 * @return float 输出
 */
float math_sinc(float x)
{
    // 分母为0则按极限求法
    if (math_abs(x) <= 2.0f * FLT_EPSILON)
    {
        return (1.0f);
    }

    return (sin(x) / x);
}

/**
 * @brief 将浮点数映射到整型
 *
 * @param x 浮点数
 * @param float_min 浮点数最小值
 * @param float_max 浮点数最大值
 * @param fnt_min 整型最小值
 * @param int_max 整型最大值
 * @return int32_t 整型
 */
int32_t math_float_to_int(float x, float float_min, float float_max, int32_t int_min, int32_t int_max)
{
    float tmp = (x - float_min) / (float_max - float_min);
    int32_t out = tmp * (float) (int_max - int_min) + int_min;
    return (out);
}

/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param int_min 整型最小值
 * @param int_max 整型最大值
 * @param float_min 浮点数最小值
 * @param float_max 浮点数最大值
 * @return float 浮点数
 */
float math_int_to_float(int32_t x, int32_t int_min, int32_t int_max, float float_min, float float_max)
{
    float tmp = (float) (x - int_min) / (float) (int_max - int_min);
    float out = tmp * (float_max - float_min) + float_min;
    return (out);
}

/************************ COPYRIGHT(C) HNUST-DUST **************************/
