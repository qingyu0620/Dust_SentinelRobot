/**
 * @file Timer.hpp
 * @author qingyu
 *
 * @brief Timer 使用教程
 * @note 1. 创建定时器：Timer timer(100); // 周期为 100 个 Tick
 * @note 2. 在周期任务中调用：timer.Tick([&]() { 每个 Tick 执行逻辑; });
 * @note 3. 使用 IsFinish() 判断周期是否结束
 * @note 4. 调用 SetTimerPeriod(new_period) 在下一周期更新周期
 * @note 5. 调用 Finish() 可立即结束当前周期
 *
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
    explicit Timer(uint32_t period) : period_(period > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(period)), 
        is_period_finish_(false), count_(0) {}
        
    ~Timer() = default;

    /**
     * @brief 定时器节拍函数
     * 
     * @note 在一个周期内持续执行回调，周期结束后自动复位计数
     * @tparam T 可调用对象类型
     * @param func 每个节拍执行的回调函数
     */
    template<typename T>
    void Tick(T && func) 
    {
        if (is_period_finish_) {
            if (is_period_changed_) period_ = period_buffer_, is_period_changed_ = false;

            is_period_finish_ = false;
        }
        
        if (count_ < period_) {
            func();
            count_++;
        } else {
            is_period_finish_ = true;
            Reset();
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
     * @brief 设置首次触发标志
     * 
     * @param flag 标志位
     */
    inline void SetFirstFlag(bool flag) { is_first_tick_flag_ = flag; }

    /**
     * @brief 获取当前计数值
     * 
     * @return uint16_t 当前计数器值
     */
    inline uint16_t GetTimerCounter() { return count_; }

    /**
     * @brief 获取首次触发标志
     * 
     * @return bool 首次触发标志
     */
    inline bool GetFirstFlag() { return is_first_tick_flag_; }

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
        is_first_tick_flag_ = true;
        is_period_finish_ = true;

        count_ = 0;
    }

private:
    bool is_first_tick_flag_ = true;

    bool is_period_changed_ = false;
    bool is_period_finish_ = false;
    uint16_t period_buffer_ = 0;
    uint16_t period_ = 0;

    uint16_t count_ = 0;
};




















