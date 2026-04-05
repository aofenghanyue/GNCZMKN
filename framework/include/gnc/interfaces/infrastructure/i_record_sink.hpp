/**
 * @file i_record_sink.hpp
 * @brief 数据记录后端接口
 */
#pragma once

#include <string>
#include <vector>

namespace gnc::interfaces {

class IRecordSink {
public:
    virtual ~IRecordSink() = default;

    virtual bool open(const std::string& session_name,
                      const std::string& output_dir) = 0;
    virtual void writeHeader(const std::vector<std::string>& column_names) = 0;
    virtual void writeRow(const std::vector<double>& values) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
};

} // namespace gnc::interfaces
