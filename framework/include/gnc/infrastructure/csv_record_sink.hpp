/**
 * @file csv_record_sink.hpp
 * @brief CSV 格式数据记录后端
 */
#pragma once

#include "gnc/common/logger.hpp"
#include "gnc/interfaces/i_record_sink.hpp"

#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace gnc::core {

class CsvRecordSink : public interfaces::IRecordSink {
public:
    void setPrecision(int precision) {
        precision_ = precision;
    }

    void setFlushEveryStep(bool flush) {
        flush_every_step_ = flush;
    }

    bool open(const std::string& session_name,
              const std::string& output_dir) override {
        const std::string filepath = output_dir + "/" + session_name + ".csv";
        file_.open(filepath);
        if (!file_.is_open()) {
            LOG_ERROR("CsvRecordSink failed to open output file '{}' in directory '{}'. Please ensure the directory exists and is writable.",
                      filepath, output_dir);
            return false;
        }
        LOG_INFO("CsvRecordSink recording to '{}'", filepath);
        return true;
    }

    void writeHeader(const std::vector<std::string>& column_names) override {
        if (!file_.is_open()) {
            return;
        }
        for (size_t i = 0; i < column_names.size(); ++i) {
            if (i > 0) file_ << ",";
            file_ << column_names[i];
        }
        file_ << "\n";
        if (flush_every_step_) {
            file_.flush();
        }
    }

    void writeRow(const std::vector<double>& values) override {
        if (!file_.is_open()) {
            return;
        }
        file_ << std::setprecision(precision_);
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) file_ << ",";
            file_ << values[i];
        }
        file_ << "\n";
        if (flush_every_step_) {
            file_.flush();
        }
    }

    void close() override {
        if (!file_.is_open()) {
            return;
        }
        file_.flush();
        file_.close();
        LOG_INFO("CsvRecordSink file closed");
    }

    bool isOpen() const override {
        return file_.is_open();
    }

private:
    std::ofstream file_;
    int precision_ = 12;
    bool flush_every_step_ = false;
};

} // namespace gnc::core
