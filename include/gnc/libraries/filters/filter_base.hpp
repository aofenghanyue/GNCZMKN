/**
 * @file filter_base.hpp
 * @brief 滤波器基类接口
 */
#pragma once

namespace gnc::libraries::filters {

/**
 * @brief 滤波器接口
 */
class IFilter {
public:
    virtual ~IFilter() = default;
    
    /**
     * @brief 滤波处理
     * @param input 输入采样
     * @return 滤波后输出
     */
    virtual double filter(double input) = 0;
    
    /**
     * @brief 重置滤波器状态
     */
    virtual void reset() = 0;
};

/**
 * @brief 滤波器类型
 */
enum class FilterType {
    Lowpass,   ///< 低通
    Highpass,  ///< 高通
    Bandpass,  ///< 带通
    Bandstop   ///< 带阻/陷波
};

} // namespace gnc::libraries::filters
