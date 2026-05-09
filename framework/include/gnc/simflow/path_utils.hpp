#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>

namespace gnc::simflow {

inline bool startsWith(const std::string& text, const std::string& prefix) {
    return text.rfind(prefix, 0) == 0;
}

inline std::filesystem::path findRepoRoot(const std::filesystem::path& anchor) {
    std::error_code ec;
    std::filesystem::path current =
        anchor.has_parent_path() ? anchor.parent_path()
                                 : std::filesystem::current_path(ec);
    if (ec) {
        current = ".";
    }
    current = current.lexically_normal();
    while (!current.empty()) {
        ec.clear();
        const bool has_cmake =
            std::filesystem::exists(current / "CMakeLists.txt", ec) && !ec;
        ec.clear();
        const bool has_framework =
            std::filesystem::exists(current / "framework", ec) && !ec;
        if (has_cmake && has_framework) {
            return current;
        }
        const auto parent = current.parent_path();
        if (parent == current) {
            break;
        }
        current = parent;
    }
    ec.clear();
    return std::filesystem::current_path(ec).lexically_normal();
}

inline std::filesystem::path findProjectRoot(const std::filesystem::path& file,
                                             const std::filesystem::path& repo_root) {
    std::error_code ec;
    const auto relative = std::filesystem::relative(file, repo_root, ec);
    if (ec) {
        return {};
    }
    auto it = relative.begin();
    if (it == relative.end() || it->generic_string() != "user") {
        return {};
    }
    ++it;
    if (it == relative.end()) {
        return {};
    }
    const auto project_name = it->generic_string();
    if (project_name.empty() || project_name == "data" ||
        project_name == "outputs") {
        return {};
    }
    const auto candidate = repo_root / "user" / project_name;
    ec.clear();
    if (!std::filesystem::is_directory(candidate, ec) || ec) {
        return {};
    }
    return candidate.lexically_normal();
}

inline std::filesystem::path resolvePath(const std::string& value,
                                         const std::filesystem::path& anchor_file,
                                         const std::filesystem::path& repo_root,
                                         const std::filesystem::path& project_root) {
    const auto strip = [](const std::string& input, const std::string& prefix) {
        return input.substr(prefix.size());
    };

    std::filesystem::path resolved;
    if (startsWith(value, "repo://")) {
        resolved = repo_root / strip(value, "repo://");
    } else if (startsWith(value, "project://")) {
        if (project_root.empty()) {
            throw std::runtime_error(
                "project:// path requires the simflow file to be under user/<project>.");
        }
        resolved = project_root / strip(value, "project://");
    } else if (startsWith(value, "user-data://")) {
        resolved = repo_root / "user" / "data" / strip(value, "user-data://");
    } else {
        const std::filesystem::path requested(value);
        if (requested.is_absolute()) {
            resolved = requested;
        } else {
            std::error_code ec;
            if (std::filesystem::exists(requested, ec) && !ec) {
                resolved = requested;
            } else if (!repo_root.empty()) {
                ec.clear();
                const auto repo_candidate = repo_root / requested;
                if (std::filesystem::exists(repo_candidate, ec) && !ec) {
                    resolved = repo_candidate;
                } else {
                    resolved = anchor_file.parent_path() / requested;
                }
            } else {
                resolved = anchor_file.parent_path() / requested;
            }
        }
    }

    std::error_code ec;
    resolved = std::filesystem::absolute(resolved, ec);
    if (ec) {
        resolved = std::filesystem::path(resolved);
    }
    return resolved.lexically_normal();
}

} // namespace gnc::simflow
