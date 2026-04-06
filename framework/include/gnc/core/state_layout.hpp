#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace gnc::core {

class StateLayout {
public:
    int addVariable(const std::string& name) {
        const int idx = static_cast<int>(names_.size());
        indices_[name] = idx;
        names_.push_back(name);
        return idx;
    }

    int indexOf(const std::string& name) const {
        auto it = indices_.find(name);
        return it != indices_.end() ? it->second : -1;
    }

    bool has(const std::string& name) const {
        return indices_.count(name) > 0;
    }

    int dimension() const {
        return static_cast<int>(names_.size());
    }

    const std::vector<std::string>& names() const {
        return names_;
    }

private:
    std::vector<std::string> names_;
    std::unordered_map<std::string, int> indices_;
};

} // namespace gnc::core
