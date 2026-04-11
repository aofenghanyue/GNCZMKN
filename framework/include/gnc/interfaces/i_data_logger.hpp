/**
 * @file i_data_logger.hpp
 * @brief 数据记录器接口
 */
#pragma once

#include <string>
#include <functional>

namespace gnc::interfaces {

/**
 * @brief 数据记录器接口
 * 
 * 用于记录仿真数据到文件（CSV/HDF5等）
 */
class IDataLogger {
public:
    virtual ~IDataLogger() = default;
    
    /// 开始记录会话
    virtual void beginSession(const std::string& session_name) = 0;
    
    /// 结束记录会话
    virtual void endSession() = 0;
    
    /// 记录当前时间步的数据
    virtual void logStep(double time) = 0;
    
    /// 检查是否正在记录
    virtual bool isRecording() const = 0;
};

} // namespace gnc::interfaces
