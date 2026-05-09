#pragma once

#include "gnc/common/math/interp.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace cavh {

struct CavhAeroSample {
    double lift_coefficient = 0.0;
    double drag_coefficient = 0.0;
};

class CavhAeroTable2D {
public:
    static CavhAeroTable2D loadFromCsv(const std::filesystem::path& path) {
        std::ifstream input(path);
        if (!input) {
            throw std::runtime_error("Cannot open CAVH aero table: " +
                                     path.string());
        }

        std::string line;
        size_t line_number = 0;
        bool saw_header = false;
        std::vector<Row> rows;

        while (std::getline(input, line)) {
            ++line_number;
            line = trim(line);
            if (line.empty() || line.front() == '#') {
                continue;
            }

            const auto fields = splitCsvLine(line);
            if (!saw_header) {
                if (fields.size() != 4 || fields[0] != "alpha_rad" ||
                    fields[1] != "mach" || fields[2] != "cl" ||
                    fields[3] != "cd") {
                    throw std::runtime_error(
                        "CAVH aero table header must be alpha_rad,mach,cl,cd.");
                }
                saw_header = true;
                continue;
            }

            if (fields.size() != 4) {
                throw std::runtime_error("CAVH aero table line " +
                                         std::to_string(line_number) +
                                         " must contain 4 comma-separated fields.");
            }

            rows.push_back({parseDouble(fields[0], line_number, "alpha_rad"),
                            parseDouble(fields[1], line_number, "mach"),
                            parseDouble(fields[2], line_number, "cl"),
                            parseDouble(fields[3], line_number, "cd")});
        }

        if (!saw_header) {
            throw std::runtime_error("CAVH aero table is missing a header.");
        }
        if (rows.empty()) {
            throw std::runtime_error("CAVH aero table has no data rows.");
        }

        return build(std::move(rows));
    }

    CavhAeroSample sample(double alpha_rad, double mach) const {
        if (!lift_table_ || !drag_table_) {
            throw std::runtime_error("CAVH aero table was not initialized.");
        }
        return {lift_table_->lookup(alpha_rad, mach),
                drag_table_->lookup(alpha_rad, mach)};
    }

private:
    struct Row {
        double alpha_rad = 0.0;
        double mach = 0.0;
        double cl = 0.0;
        double cd = 0.0;
    };

    CavhAeroTable2D(std::vector<double> alpha_breaks,
                    std::vector<double> mach_breaks,
                    std::vector<std::vector<double>> cl_grid,
                    std::vector<std::vector<double>> cd_grid)
        : alpha_breaks_(std::move(alpha_breaks)),
          mach_breaks_(std::move(mach_breaks)),
          lift_table_(std::make_unique<gnc::math::LookupTable2D>(
              alpha_breaks_,
              mach_breaks_,
              std::move(cl_grid))),
          drag_table_(std::make_unique<gnc::math::LookupTable2D>(
              alpha_breaks_,
              mach_breaks_,
              std::move(cd_grid))) {}

    static CavhAeroTable2D build(std::vector<Row> rows) {
        std::vector<double> alpha_breaks;
        std::vector<double> mach_breaks;
        for (const auto& row : rows) {
            alpha_breaks.push_back(row.alpha_rad);
            mach_breaks.push_back(row.mach);
        }
        sortUnique(alpha_breaks);
        sortUnique(mach_breaks);

        if (alpha_breaks.size() < 2 || mach_breaks.size() < 2) {
            throw std::runtime_error(
                "CAVH aero table requires at least 2 alpha and 2 mach nodes.");
        }

        const double missing = std::numeric_limits<double>::quiet_NaN();
        std::vector<std::vector<double>> cl_grid(
            alpha_breaks.size(), std::vector<double>(mach_breaks.size(), missing));
        std::vector<std::vector<double>> cd_grid(
            alpha_breaks.size(), std::vector<double>(mach_breaks.size(), missing));
        std::map<std::pair<double, double>, bool> seen;

        for (const auto& row : rows) {
            const auto alpha_it =
                std::find(alpha_breaks.begin(), alpha_breaks.end(), row.alpha_rad);
            const auto mach_it =
                std::find(mach_breaks.begin(), mach_breaks.end(), row.mach);
            const size_t i = static_cast<size_t>(
                std::distance(alpha_breaks.begin(), alpha_it));
            const size_t j = static_cast<size_t>(
                std::distance(mach_breaks.begin(), mach_it));

            const auto key = std::make_pair(row.alpha_rad, row.mach);
            if (seen[key]) {
                throw std::runtime_error(
                    "CAVH aero table contains a duplicate alpha/mach row.");
            }
            seen[key] = true;
            cl_grid[i][j] = row.cl;
            cd_grid[i][j] = row.cd;
        }

        for (size_t i = 0; i < alpha_breaks.size(); ++i) {
            for (size_t j = 0; j < mach_breaks.size(); ++j) {
                if (std::isnan(cl_grid[i][j]) || std::isnan(cd_grid[i][j])) {
                    throw std::runtime_error(
                        "CAVH aero table does not cover the full alpha/mach grid.");
                }
            }
        }

        return CavhAeroTable2D(std::move(alpha_breaks),
                               std::move(mach_breaks),
                               std::move(cl_grid),
                               std::move(cd_grid));
    }

    static void sortUnique(std::vector<double>& values) {
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end()), values.end());
    }

    static std::vector<std::string> splitCsvLine(const std::string& line) {
        std::vector<std::string> fields;
        std::stringstream stream(line);
        std::string field;
        while (std::getline(stream, field, ',')) {
            fields.push_back(trim(field));
        }
        return fields;
    }

    static std::string trim(const std::string& value) {
        const auto begin = std::find_if_not(
            value.begin(), value.end(), [](unsigned char c) {
                return std::isspace(c) != 0;
            });
        const auto end = std::find_if_not(
                             value.rbegin(), value.rend(), [](unsigned char c) {
                                 return std::isspace(c) != 0;
                             })
                             .base();
        if (begin >= end) {
            return {};
        }
        return std::string(begin, end);
    }

    static double parseDouble(const std::string& value,
                              size_t line_number,
                              const std::string& field_name) {
        try {
            size_t parsed = 0;
            const double result = std::stod(value, &parsed);
            if (parsed != value.size()) {
                throw std::invalid_argument("trailing characters");
            }
            return result;
        } catch (const std::exception&) {
            throw std::runtime_error("CAVH aero table line " +
                                     std::to_string(line_number) + " field " +
                                     field_name + " must be numeric.");
        }
    }

    std::vector<double> alpha_breaks_;
    std::vector<double> mach_breaks_;
    std::unique_ptr<gnc::math::LookupTable2D> lift_table_;
    std::unique_ptr<gnc::math::LookupTable2D> drag_table_;
};

} // namespace cavh
