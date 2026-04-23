#pragma once

#include "gnc/services/coordinate_tree/interfaces/i_coordinate_tree_spec.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace gnc::services::coordinate_tree::internal {

class CoordinateTreeSpecRegistry {
public:
    void registerSpec(std::unique_ptr<ICoordinateTreeSpec> spec) {
        if (!spec) {
            throw std::runtime_error("Cannot register a null coordinate-tree spec.");
        }

        const std::string spec_id(spec->id());
        if (spec_id.empty()) {
            throw std::runtime_error("Coordinate-tree spec id cannot be empty.");
        }
        if (specs_.count(spec_id) > 0) {
            throw std::runtime_error("Coordinate-tree spec '" + spec_id +
                                     "' is already registered.");
        }

        specs_.emplace(spec_id, std::move(spec));
    }

    const ICoordinateTreeSpec* findSpec(std::string_view id) const {
        const auto it = specs_.find(std::string(id));
        if (it == specs_.end()) {
            return nullptr;
        }
        return it->second.get();
    }

    std::vector<std::string> listSpecIds() const {
        std::vector<std::string> ids;
        ids.reserve(specs_.size());
        for (const auto& [id, _] : specs_) {
            ids.push_back(id);
        }
        std::sort(ids.begin(), ids.end());
        return ids;
    }

private:
    std::unordered_map<std::string, std::unique_ptr<ICoordinateTreeSpec>> specs_;
};

} // namespace gnc::services::coordinate_tree::internal
