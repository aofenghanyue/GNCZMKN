#pragma once

#include <iomanip>
#include <sstream>
#include <string>

namespace gnc::simflow {

inline std::string zeroPaddedCaseIndex(size_t case_index) {
    std::ostringstream out;
    out << std::setw(6) << std::setfill('0') << case_index;
    return out.str();
}

inline std::string formatCaseDirectory(const std::string& pattern,
                                       size_t case_index,
                                       const std::string& case_id = {}) {
    std::string result = pattern.empty() ? "case_{case_index}" : pattern;
    const auto replace_all = [](std::string& text,
                                const std::string& token,
                                const std::string& value) {
        size_t pos = 0;
        while ((pos = text.find(token, pos)) != std::string::npos) {
            text.replace(pos, token.size(), value);
            pos += value.size();
        }
    };
    replace_all(result, "{case_index}", zeroPaddedCaseIndex(case_index));
    replace_all(result, "{case_id}", case_id.empty() ? zeroPaddedCaseIndex(case_index)
                                                     : case_id);
    return result;
}

} // namespace gnc::simflow
