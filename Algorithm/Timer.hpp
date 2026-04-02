/**
 * @file Timer.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-03-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include <cstdint>

/**
 * @brief Timer
 * 
 */
class Timer final
{
public:
    /**
     * @brief 构造定时器对象
     * 
     * @param period 定时器周期，最大为 UINT16_MAX
     */
    Timer(uint32_t period) : period_(period),  is_period_finish_(false), count_(0) {}
    ~Timer() = default;

    /**
     * @brief 节拍触发函数
     * 
     * @note 在一个周期内持续执行回调，周期结束后自动复位计数
     * @tparam T 可调用对象类型
     * @param func 每个节拍执行的回调函数
     */
    template<typename T>
    void Tick(T && func) 
    {
        if (is_period_finish_) 
        {
            if (is_period_changed_) {
                period_ = period_buffer_, is_period_changed_ = false;
            }
            is_period_finish_ = false;
        }
        
        if (count_ < period_) {
            func();
            count_++;
        } else {
            Finish();
        }
    }

    /**
     * @brief 定时触发函数
     * 
     * @note 周期内只计数不执行，满周期后执行一次回调并自动复位
     * @tparam T 可调用对象类型
     * @param func 满周期时执行的回调函数
     */
    template<typename T>
    void Clock(T && func)
    {
        if (is_period_finish_) 
        {
            if (is_period_changed_) {
                period_ = period_buffer_, is_period_changed_ = false;
            }
            is_period_finish_ = false;
        }
        if (count_ < period_) {
            count_++;
        } else {
            func();
            Finish();
        }
    }

    /**
     * @brief 设置定时器周期
     * 
     * @note 在下一次周期时更新
     * @param period 
     */
    inline void SetTimerPeriod(uint16_t period) 
    {
        if (is_period_finish_) {
            period_ = period;
            is_period_changed_ = false;
        } else {
            period_buffer_ = period;
            is_period_changed_ = true;
        }
    }
    /**
     * @brief 获取当前计数值
     * 
     * @return uint16_t 当前计数器值
     */
    inline uint16_t GetTimerCounter() { return count_; }

    /**
     * @brief 查询当前周期是否完成
     * 
     * @return bool 周期完成状态
     */
    inline bool IsFinish() { return is_period_finish_; }

    /**
     * @brief 复位计数器
     */
    inline void Reset() { count_ = 0; }

    /**
     * @brief 立即结束当前周期
     * 
     * @note 调用后会置位完成标志并清零计数
     */
    inline void Finish() 
    {
        is_period_finish_ = true;
        count_ = 0;
    }

private:
    bool is_period_changed_ = false;
    bool is_period_finish_ = false;
    uint32_t period_buffer_ = 0;
    uint32_t period_ = 0;

    uint32_t count_ = 0;
};
