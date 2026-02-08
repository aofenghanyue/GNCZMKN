/**
 * @file data_logger.hpp
 * @brief 数据记录器实现
 * 
 * 支持：
 * - 注册任意数据话题
 * - 输出到 CSV 格式
 * - 类型安全的数据采集
 */
#pragma once

#include "gnc/interfaces/infrastructure/i_data_logger.hpp"
#include "gnc/common/logger.hpp"
#include <fstream>
#include <vector>
#include <functional>
#include <memory>
#include <sstream>
#include <iomanip>

namespace gnc::core {

/**
 * @brief 数据话题基类
 */
class DataTopicBase {
public:
    virtual ~DataTopicBase() = default;
    virtual std::string getHeader() const = 0;
    virtual std::string getData() const = 0;
};

/**
 * @brief 类型安全的数据话题
 * @tparam T 数据类型
 */
template<typename T>
class DataTopic : public DataTopicBase {
public:
    using GetterFunc = std::function<const T&()>;
    using FormatterFunc = std::function<std::string(const T&)>;
    
    DataTopic(const std::string& name, 
              GetterFunc getter,
              const std::vector<std::string>& field_names,
              FormatterFunc formatter)
        : name_(name)
        , getter_(std::move(getter))
        , field_names_(field_names)
        , formatter_(std::move(formatter)) {}
    
    std::string getHeader() const override {
        std::ostringstream oss;
        for (size_t i = 0; i < field_names_.size(); ++i) {
            if (i > 0) oss << ",";
            oss << name_ << "." << field_names_[i];
        }
        return oss.str();
    }
    
    std::string getData() const override {
        return formatter_(getter_());
    }
    
private:
    std::string name_;
    GetterFunc getter_;
    std::vector<std::string> field_names_;
    FormatterFunc formatter_;
};

/**
 * @brief CSV 数据记录器
 * 
 * 使用示例：
 * @code
 * CsvDataLogger logger;
 * 
 * // 注册话题
 * logger.registerTopic<NavState>("nav", 
 *     [&nav]() -> const NavState& { return nav.getNavState(); },
 *     {"pos.x", "pos.y", "pos.z", "vel.x", "vel.y", "vel.z"},
 *     [](const NavState& s) {
 *         return fmt("{},{},{},{},{},{}", 
 *                    s.position.x, s.position.y, s.position.z,
 *                    s.velocity.x, s.velocity.y, s.velocity.z);
 *     });
 * 
 * logger.beginSession("sim_001");
 * // 仿真循环中
 * logger.logStep(current_time);
 * // 结束
 * logger.endSession();
 * @endcode
 */
class CsvDataLogger : public interfaces::IDataLogger {
public:
    CsvDataLogger() = default;
    ~CsvDataLogger() override {
        if (is_recording_) {
            endSession();
        }
    }
    
    /**
     * @brief 注册数据话题
     * @tparam T 数据类型
     * @param name 话题名称
     * @param getter 获取数据的函数
     * @param field_names 字段名列表
     * @param formatter 格式化函数（返回CSV格式字符串）
     */
    template<typename T>
    void registerTopic(const std::string& name,
                       std::function<const T&()> getter,
                       const std::vector<std::string>& field_names,
                       std::function<std::string(const T&)> formatter) {
        topics_.push_back(std::make_unique<DataTopic<T>>(
            name, std::move(getter), field_names, std::move(formatter)
        ));
        LOG_INFO("Registered data topic: {} ({} fields)", name, field_names.size());
    }
    
    /**
     * @brief 注册简单数值话题
     */
    void registerScalar(const std::string& name, std::function<double()> getter) {
        auto wrapper = [getter]() -> const double& {
            static double val;
            val = getter();
            return val;
        };
        registerTopic<double>(name, wrapper, {name}, 
            [](const double& v) { 
                std::ostringstream oss;
                oss << std::setprecision(12) << v;
                return oss.str();
            });
    }
    
    // --- IDataLogger 接口 ---
    
    void beginSession(const std::string& session_name) override {
        if (is_recording_) {
            endSession();
        }
        
        std::string filename = session_name + ".csv";
        file_.open(filename);
        
        if (!file_.is_open()) {
            LOG_ERROR("Failed to open log file: {}", filename);
            return;
        }
        
        // 写入表头
        file_ << "time";
        for (const auto& topic : topics_) {
            file_ << "," << topic->getHeader();
        }
        file_ << "\n";
        
        is_recording_ = true;
        LOG_INFO("Data logging started: {}", filename);
    }
    
    void endSession() override {
        if (file_.is_open()) {
            file_.close();
            LOG_INFO("Data logging ended");
        }
        is_recording_ = false;
    }
    
    void logStep(double time) override {
        if (!is_recording_ || !file_.is_open()) return;
        
        file_ << std::setprecision(12) << time;
        for (const auto& topic : topics_) {
            file_ << "," << topic->getData();
        }
        file_ << "\n";
    }
    
    bool isRecording() const override {
        return is_recording_;
    }
    
    /// 设置是否每步刷新（影响性能但保证数据安全）
    void setFlushEveryStep(bool flush) {
        flush_every_step_ = flush;
    }
    
private:
    std::vector<std::unique_ptr<DataTopicBase>> topics_;
    std::ofstream file_;
    bool is_recording_ = false;
    bool flush_every_step_ = false;
};

} // namespace gnc::core
