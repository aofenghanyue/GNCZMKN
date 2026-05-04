#pragma once

#include "gnc/core/config_manager.hpp"
#include "gnc/core/config_reader.hpp"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace gnc::vehicle::common::assets {

namespace fs = std::filesystem;

inline std::string getConfiguredJsonAssetFile(
    const gnc::core::ConfigNode& config,
    const std::string& config_path = "config") {
    gnc::core::ConfigReader reader(config, config_path);
    if (!reader.has("asset_file")) {
        return "";
    }
    return reader.optionalString("asset_file", "");
}

inline bool hasConfiguredJsonAssetFile(
    const gnc::core::ConfigNode& config,
    const std::string& config_path = "config") {
    return !getConfiguredJsonAssetFile(config, config_path).empty();
}

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

inline bool startsWith(const std::string& text, const std::string& prefix) {
    return text.rfind(prefix, 0) == 0;
}

inline std::string trim(std::string text) {
    while (!text.empty() &&
           std::isspace(static_cast<unsigned char>(text.back()))) {
        text.pop_back();
    }
    size_t first = 0;
    while (first < text.size() &&
           std::isspace(static_cast<unsigned char>(text[first]))) {
        ++first;
    }
    return text.substr(first);
}

inline fs::path findRepoRootFromCurrent() {
    std::error_code ec;
    fs::path current = fs::current_path(ec);
    if (ec) {
        current = ".";
    }
    current = current.lexically_normal();

    while (!current.empty()) {
        ec.clear();
        const bool has_cmake = fs::exists(current / "CMakeLists.txt", ec) && !ec;
        ec.clear();
        const bool has_framework = fs::exists(current / "framework", ec) && !ec;
        if (has_cmake && has_framework) {
            return current.lexically_normal();
        }

        const fs::path parent = current.parent_path();
        if (parent == current) {
            break;
        }
        current = parent;
    }

    ec.clear();
    return fs::current_path(ec).lexically_normal();
}

inline std::string readActiveProjectName(const fs::path& repo_root) {
    std::ifstream stream(repo_root / "user/active_project");
    if (!stream.is_open()) {
        return "";
    }
    std::string value;
    std::getline(stream, value);
    return trim(value);
}

inline fs::path pathAfterScheme(const std::string& value,
                                const std::string& scheme) {
    return fs::path(value.substr(scheme.size())).lexically_normal();
}

inline fs::path resolveUriJsonAssetPath(const std::string& configured_path,
                                        std::vector<fs::path>& searched_paths) {
    constexpr const char* kRepoScheme = "repo://";
    constexpr const char* kProjectScheme = "project://";
    constexpr const char* kUserDataScheme = "user-data://";

    const fs::path repo_root = findRepoRootFromCurrent();
    fs::path candidate;
    if (startsWith(configured_path, kRepoScheme)) {
        candidate = repo_root / pathAfterScheme(configured_path, kRepoScheme);
    } else if (startsWith(configured_path, kUserDataScheme)) {
        candidate = repo_root / "user/data" /
                    pathAfterScheme(configured_path, kUserDataScheme);
    } else if (startsWith(configured_path, kProjectScheme)) {
        const std::string project_name = readActiveProjectName(repo_root);
        if (project_name.empty() ||
            project_name.find('/') != std::string::npos ||
            project_name.find('\\') != std::string::npos) {
            throw std::runtime_error(
                "Asset file '" + configured_path +
                "' uses project:// but user/active_project is missing or invalid.");
        }
        candidate = repo_root / "user" / project_name /
                    pathAfterScheme(configured_path, kProjectScheme);
    } else {
        return {};
    }

    searched_paths.push_back(candidate);
    std::error_code ec;
    if (fs::exists(candidate, ec) && !ec) {
        return fs::absolute(candidate, ec).lexically_normal();
    }
    return {};
}

inline fs::path resolveJsonAssetPath(const std::string& configured_path) {
    std::vector<fs::path> searched_paths;
    std::error_code ec;

    if (startsWith(configured_path, "repo://") ||
        startsWith(configured_path, "project://") ||
        startsWith(configured_path, "user-data://")) {
        fs::path resolved = resolveUriJsonAssetPath(configured_path, searched_paths);
        if (!resolved.empty()) {
            return resolved;
        }
        throw std::runtime_error("Asset file '" + configured_path +
                                 "' was not found. Checked: " +
                                 joinPaths(searched_paths));
    }

    const fs::path requested(configured_path);
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

inline fs::path resolveConfiguredJsonAssetPath(
    const gnc::core::ConfigNode& config,
    const std::string& config_path = "config") {
    const std::string asset_file = getConfiguredJsonAssetFile(config, config_path);
    if (asset_file.empty()) {
        return {};
    }
    return resolveJsonAssetPath(asset_file);
}

inline gnc::core::ConfigNode loadConfiguredJsonAsset(
    const gnc::core::ConfigNode& config,
    const std::string& type_name,
    const std::string& config_path = "config") {
    if (!hasConfiguredJsonAssetFile(config, config_path)) {
        return config;
    }

    const fs::path resolved_path = resolveConfiguredJsonAssetPath(config, config_path);
    gnc::core::ConfigManager manager;
    if (!manager.loadFromFile(resolved_path.string())) {
        throw std::runtime_error(type_name + " failed to load asset file '" +
                                 resolved_path.generic_string() + "'.");
    }
    return manager.root();
}

} // namespace gnc::vehicle::common::assets
