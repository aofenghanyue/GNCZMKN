/**
 * @file auto_data_logger.hpp
 * @brief 配置驱动的自动数据记录器
 */
#pragma once

#include "gnc/common/logger.hpp"
#include "gnc/core/component_registry.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/csv_record_sink.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/interfaces/infrastructure/i_record_sink.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

namespace gnc::core {

class AutoDataLogger {
public:
    AutoDataLogger() = default;

    ~AutoDataLogger() {
        stop();
    }

    bool initialize(const ConfigNode& config, const ComponentRegistry& registry) {
        stop();
        active_fields_.clear();
        row_buffer_.clear();
        output_dir_.clear();
        enabled_ = false;
        sink_.reset();

        if (config.isNull()) {
            LOG_INFO("AutoDataLogger disabled because no 'outputs' configuration was provided");
            return true;
        }

        if (config.has("enabled") && !config["enabled"].asBool(true)) {
            LOG_INFO("AutoDataLogger disabled by configuration");
            return true;
        }

        output_dir_ = resolveOutputDirectory(config["directory"].asString("user/outputs"));
        if (!ensureOutputDirectory(output_dir_)) {
            return false;
        }

        const std::string format = config["format"].asString("csv");
        const std::string session_name = config["session_name"].asString("simulation_data");
        const int precision = config["precision"].asInt(12);
        const bool flush_every_step = config["flush_every_step"].asBool(false);

        RecordRule rule = parseRecordRule(config);
        std::vector<std::string> excludes = parseExcludeList(config);
        discoverFields(registry, rule, excludes);

        enabled_ = true;

        if (active_fields_.empty()) {
            LOG_WARNING("AutoDataLogger found no observable fields matching the configured record rule. No CSV file will be produced. Please check component names, field names, and whether target components implement IObservable.");
            return true;
        }

        if (format == "csv") {
            auto csv_sink = std::make_unique<CsvRecordSink>();
            csv_sink->setPrecision(precision);
            csv_sink->setFlushEveryStep(flush_every_step);
            sink_ = std::move(csv_sink);
        } else {
            LOG_ERROR("AutoDataLogger does not support output format '{}'. Please use 'csv' or update the configuration.", format);
            enabled_ = false;
            return false;
        }

        if (!sink_->open(session_name, output_dir_)) {
            enabled_ = false;
            return false;
        }

        std::vector<std::string> headers;
        headers.reserve(active_fields_.size() + 1);
        headers.push_back("time");
        for (const auto& field : active_fields_) {
            headers.push_back(field.column_name);
        }
        sink_->writeHeader(headers);
        row_buffer_.reserve(active_fields_.size() + 1);

        LOG_INFO("AutoDataLogger enabled with {} field(s). Output directory: '{}'", active_fields_.size(), output_dir_);
        return true;
    }

    void recordStep(double time) {
        if (!enabled_ || !sink_ || !sink_->isOpen()) {
            return;
        }

        row_buffer_.clear();
        row_buffer_.push_back(time);
        for (const auto& field : active_fields_) {
            row_buffer_.push_back(field.getter());
        }
        sink_->writeRow(row_buffer_);
    }

    void stop() {
        if (sink_ && sink_->isOpen()) {
            sink_->close();
        }
    }

    bool isEnabled() const {
        return enabled_;
    }

    size_t getFieldCount() const {
        return active_fields_.size();
    }

    const std::string& getOutputDir() const {
        return output_dir_;
    }

private:
    struct ActiveField {
        std::string column_name;
        std::function<double()> getter;
    };

    struct RecordRule {
        enum class Mode {
            None,
            All,
            Components,
            Detailed
        };

        struct ComponentRule {
            std::string component_name;
            bool all_fields = false;
            std::vector<std::string> field_prefixes;
        };

        Mode mode = Mode::None;
        std::vector<std::string> component_names;
        std::vector<ComponentRule> component_rules;
    };

    static std::string makeTimestampString() {
        const auto now = std::chrono::system_clock::now();
        const auto time_value = std::chrono::system_clock::to_time_t(now);
        std::tm tm_value{};
#ifdef _WIN32
        localtime_s(&tm_value, &time_value);
#else
        localtime_r(&time_value, &tm_value);
#endif
        char buffer[20];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H%M%S", &tm_value);
        return buffer;
    }

    static std::string resolveOutputDirectory(std::string output_dir) {
        const std::string token = "{timestamp}";
        const auto pos = output_dir.find(token);
        if (pos != std::string::npos) {
            output_dir.replace(pos, token.size(), makeTimestampString());
        }
        return output_dir;
    }

    static bool ensureOutputDirectory(const std::string& output_dir) {
        if (output_dir.empty()) {
            LOG_ERROR("AutoDataLogger received an empty output directory. Please configure a valid outputs.directory value.");
            return false;
        }

        std::error_code ec;
        std::filesystem::create_directories(output_dir, ec);
        if (ec) {
            LOG_ERROR("AutoDataLogger failed to create output directory '{}'. Please ensure the path is valid and writable. Error: {}",
                      output_dir, ec.message());
            return false;
        }
        return true;
    }

    RecordRule parseRecordRule(const ConfigNode& config) const {
        RecordRule rule;
        const auto& record = config["record"];

        if (record.isNull()) {
            rule.mode = RecordRule::Mode::All;
            return rule;
        }

        if (record.isString()) {
            if (record.asString() == "all") {
                rule.mode = RecordRule::Mode::All;
            }
            return rule;
        }

        if (record.isArray()) {
            rule.mode = RecordRule::Mode::Components;
            for (size_t i = 0; i < record.size(); ++i) {
                if (record[i].isString()) {
                    rule.component_names.push_back(record[i].asString());
                }
            }
            return rule;
        }

        if (record.isObject()) {
            rule.mode = RecordRule::Mode::Detailed;
            for (const auto& [component_name, component_rule_node] : record) {
                RecordRule::ComponentRule component_rule;
                component_rule.component_name = component_name;
                if (component_rule_node.isString() && component_rule_node.asString() == "all") {
                    component_rule.all_fields = true;
                } else if (component_rule_node.isArray()) {
                    for (size_t i = 0; i < component_rule_node.size(); ++i) {
                        if (component_rule_node[i].isString()) {
                            component_rule.field_prefixes.push_back(component_rule_node[i].asString());
                        }
                    }
                }
                rule.component_rules.push_back(std::move(component_rule));
            }
        }

        return rule;
    }

    std::vector<std::string> parseExcludeList(const ConfigNode& config) const {
        std::vector<std::string> excludes;
        const auto& exclude_node = config["exclude"];
        if (!exclude_node.isArray()) {
            return excludes;
        }

        for (size_t i = 0; i < exclude_node.size(); ++i) {
            if (exclude_node[i].isString()) {
                excludes.push_back(exclude_node[i].asString());
            }
        }
        return excludes;
    }

    void discoverFields(const ComponentRegistry& registry,
                        const RecordRule& rule,
                        const std::vector<std::string>& excludes) {
        active_fields_.clear();

        for (auto* component : registry.getAllComponents()) {
            auto* observable = dynamic_cast<interfaces::IObservable*>(component);
            if (!observable) {
                continue;
            }

            bool should_record_component = false;
            bool record_all_fields = false;
            std::vector<std::string> requested_prefixes;
            const std::string& component_name = component->getName();

            switch (rule.mode) {
            case RecordRule::Mode::None:
                continue;
            case RecordRule::Mode::All:
                should_record_component = true;
                record_all_fields = true;
                break;
            case RecordRule::Mode::Components:
                should_record_component = std::find(rule.component_names.begin(),
                                                    rule.component_names.end(),
                                                    component_name) != rule.component_names.end();
                record_all_fields = should_record_component;
                break;
            case RecordRule::Mode::Detailed:
                for (const auto& component_rule : rule.component_rules) {
                    if (component_rule.component_name == component_name) {
                        should_record_component = true;
                        record_all_fields = component_rule.all_fields;
                        requested_prefixes = component_rule.field_prefixes;
                        break;
                    }
                }
                break;
            }

            if (!should_record_component) {
                continue;
            }

            const auto fields = observable->getObservableFields();
            for (const auto& field : fields) {
                if (!record_all_fields && !matchesFieldPrefixes(field.name, requested_prefixes)) {
                    continue;
                }

                const std::string full_name = component_name + "." + field.name;
                if (isExcluded(full_name, field.name, excludes)) {
                    continue;
                }

                active_fields_.push_back({full_name, field.getter});
            }
        }
    }

    static bool matchesFieldPrefixes(const std::string& field_name,
                                     const std::vector<std::string>& prefixes) {
        for (const auto& prefix : prefixes) {
            if (field_name == prefix) {
                return true;
            }
            const std::string nested_prefix = prefix + ".";
            if (field_name.size() > nested_prefix.size() &&
                field_name.compare(0, nested_prefix.size(), nested_prefix) == 0) {
                return true;
            }
        }
        return false;
    }

    static bool isExcluded(const std::string& full_name,
                           const std::string& field_name,
                           const std::vector<std::string>& excludes) {
        for (const auto& pattern : excludes) {
            if (pattern.size() > 2 && pattern[0] == '*' && pattern[1] == '.') {
                const std::string suffix = pattern.substr(2);
                if (field_name == suffix) {
                    return true;
                }
                const std::string suffix_with_dot = "." + suffix;
                if (field_name.size() > suffix_with_dot.size() &&
                    field_name.compare(field_name.size() - suffix_with_dot.size(),
                                       suffix_with_dot.size(),
                                       suffix_with_dot) == 0) {
                    return true;
                }
            }

            if (full_name == pattern) {
                return true;
            }
        }
        return false;
    }

    std::unique_ptr<interfaces::IRecordSink> sink_;
    std::vector<ActiveField> active_fields_;
    std::vector<double> row_buffer_;
    std::string output_dir_;
    bool enabled_ = false;
};

} // namespace gnc::core
