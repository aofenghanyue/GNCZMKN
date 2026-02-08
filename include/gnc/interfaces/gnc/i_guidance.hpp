/**
 * @file i_guidance.hpp
 * @brief 制导接口
 */
#pragma once

#include "gnc/interfaces/data_types.hpp"

namespace gnc::interfaces {

/**
 * @brief 制导算法接口
 */
class IGuidance {
public:
    virtual ~IGuidance() = default;
    
    /// 获取当前制导指令
    virtual const GuidanceCommand& getGuidanceCommand() const = 0;
    
    /// 设置目标位置
    virtual void setTarget(const Vector3d& target_position) = 0;
    
    /// 制导是否有效
    virtual bool isActive() const = 0;
};

} // namespace gnc::interfaces
