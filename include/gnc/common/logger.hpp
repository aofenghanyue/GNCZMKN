/**
 * @file logger.hpp
 * @brief 日志接口
 */
#pragma once

#include <string>
#include <iostream>
#include <sstream>

namespace gnc {

/// 日志级别
enum class LogLevel {
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical
};

/// 简易日志类
class Logger {
public:
    static Logger& instance() {
        static Logger inst;
        return inst;
    }
    
    void setLevel(LogLevel level) { level_ = level; }
    
    template<typename... Args>
    void log(LogLevel level, const char* fmt, Args&&... args) {
        if (level < level_) return;
        
        std::ostringstream oss;
        oss << "[" << levelToString(level) << "] ";
        formatTo(oss, fmt, std::forward<Args>(args)...);
        std::cout << oss.str() << std::endl;
    }
    
private:
    Logger() = default;
    LogLevel level_ = LogLevel::Info;
    
    static const char* levelToString(LogLevel level) {
        switch (level) {
            case LogLevel::Trace:    return "TRACE";
            case LogLevel::Debug:    return "DEBUG";
            case LogLevel::Info:     return "INFO";
            case LogLevel::Warning:  return "WARN";
            case LogLevel::Error:    return "ERROR";
            case LogLevel::Critical: return "CRIT";
            default: return "?";
        }
    }
    
    // 简单格式化（占位实现）
    void formatTo(std::ostringstream& oss, const char* fmt) {
        oss << fmt;
    }
    
    template<typename T, typename... Rest>
    void formatTo(std::ostringstream& oss, const char* fmt, T&& val, Rest&&... rest) {
        while (*fmt) {
            if (*fmt == '{' && *(fmt+1) == '}') {
                oss << val;
                formatTo(oss, fmt + 2, std::forward<Rest>(rest)...);
                return;
            }
            oss << *fmt++;
        }
    }
};

// 便捷宏
#define LOG_TRACE(...)    gnc::Logger::instance().log(gnc::LogLevel::Trace, __VA_ARGS__)
#define LOG_DEBUG(...)    gnc::Logger::instance().log(gnc::LogLevel::Debug, __VA_ARGS__)
#define LOG_INFO(...)     gnc::Logger::instance().log(gnc::LogLevel::Info, __VA_ARGS__)
#define LOG_WARNING(...)  gnc::Logger::instance().log(gnc::LogLevel::Warning, __VA_ARGS__)
#define LOG_ERROR(...)    gnc::Logger::instance().log(gnc::LogLevel::Error, __VA_ARGS__)
#define LOG_CRITICAL(...) gnc::Logger::instance().log(gnc::LogLevel::Critical, __VA_ARGS__)

} // namespace gnc
