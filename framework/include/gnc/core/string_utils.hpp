/**
 * @file string_utils.hpp
 * @brief 字符串工具函数
 */
#pragma once

#include <algorithm>
#include <string>
#include <vector>

namespace gnc::core {

inline int editDistance(const std::string& a, const std::string& b) {
    const int m = static_cast<int>(a.size());
    const int n = static_cast<int>(b.size());

    std::vector<int> prev(n + 1);
    std::vector<int> curr(n + 1);

    for (int j = 0; j <= n; ++j) {
        prev[j] = j;
    }

    for (int i = 1; i <= m; ++i) {
        curr[0] = i;
        for (int j = 1; j <= n; ++j) {
            if (a[i - 1] == b[j - 1]) {
                curr[j] = prev[j - 1];
            } else {
                curr[j] = 1 + std::min({prev[j], curr[j - 1], prev[j - 1]});
            }
        }
        std::swap(prev, curr);
    }

    return prev[n];
}

inline std::string findClosestMatch(const std::string& target,
                                    const std::vector<std::string>& candidates,
                                    int max_distance = 3) {
    std::string best_match;
    int best_distance = max_distance + 1;

    for (const auto& candidate : candidates) {
        const int distance = editDistance(target, candidate);
        if (distance < best_distance) {
            best_distance = distance;
            best_match = candidate;
        }
    }

    if (best_distance > max_distance) {
        return "";
    }
    return best_match;
}

} // namespace gnc::core
