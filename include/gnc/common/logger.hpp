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

// 便捷函数
template<typename... Args>
inline void log_trace(const char* fmt, Args&&... args) {
    Logger::instance().log(LogLevel::Trace, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
inline void log_debug(const char* fmt, Args&&... args) {
    Logger::instance().log(LogLevel::Debug, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
inline void log_info(const char* fmt, Args&&... args) {
    Logger::instance().log(LogLevel::Info, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
inline void log_warning(const char* fmt, Args&&... args) {
    Logger::instance().log(LogLevel::Warning, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
inline void log_error(const char* fmt, Args&&... args) {
    Logger::instance().log(LogLevel::Error, fmt, std::forward<Args>(args)...);
}

template<typename... Args>
inline void log_critical(const char* fmt, Args&&... args) {
    Logger::instance().log(LogLevel::Critical, fmt, std::forward<Args>(args)...);
}

} // namespace gnc
