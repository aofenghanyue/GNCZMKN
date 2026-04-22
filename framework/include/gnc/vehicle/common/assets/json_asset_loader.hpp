#pragma once

#include "gnc/core/config_manager.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace gnc::vehicle::common::assets {

namespace fs = std::filesystem;

inline std::string joinPaths(const std::vector<fs::path>& paths) {
    std::string result;
    for (size_t i = 0; i < paths.size(); ++i) {
        if (i > 0) {
            result += ", ";
        }
        result += paths[i].lexically_normal().generic_string();
    }
    return result.empty() ? "(none)" : result;
}

inline fs::path resolveJsonAssetPath(const std::string& configured_path) {
    const fs::path requested(configured_path);
    std::vector<fs::path> searched_paths;
    std::error_code ec;

    if (requested.is_absolute()) {
        searched_paths.push_back(requested);
        if (fs::exists(requested, ec) && !ec) {
            return fs::absolute(requested, ec).lexically_normal();
        }
    } else {
        fs::path current = fs::current_path(ec);
        while (!ec && !current.empty()) {
            const fs::path candidate = current / requested;
            searched_paths.push_back(candidate);

            ec.clear();
            if (fs::exists(candidate, ec) && !ec) {
                return fs::absolute(candidate, ec).lexically_normal();
            }

            const fs::path parent = current.parent_path();
            if (parent == current) {
                break;
            }
            current = parent;
        }
    }

    throw std::runtime_error("Asset file '" + configured_path +
                             "' was not found. Checked: " +
                             joinPaths(searched_paths));
}

inline gnc::core::ConfigNode loadConfiguredJsonAsset(
    const gnc::core::ConfigNode& config,
    const std::string& type_name) {
    const std::string asset_file = config["asset_file"].asString();
    if (asset_file.empty()) {
        return config;
    }

    const fs::path resolved_path = resolveJsonAssetPath(asset_file);
    gnc::core::ConfigManager manager;
    if (!manager.loadFromFile(resolved_path.string())) {
        throw std::runtime_error(type_name + " failed to load asset file '" +
                                 resolved_path.generic_string() + "'.");
    }
    return manager.root();
}

} // namespace gnc::vehicle::common::assets
