#include "test_support.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

fs::path repoRoot() {
    return fs::path(__FILE__).lexically_normal().parent_path().parent_path();
}

std::string normalizePath(const fs::path& path) {
    return path.lexically_normal().generic_string();
}

std::string readFile(const fs::path& path) {
    std::ifstream stream(path, std::ios::binary);
    test_support::require(stream.is_open(),
                          "Failed to read file: " + normalizePath(path));

    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

std::vector<fs::path> collectFiles(const fs::path& root) {
    std::vector<fs::path> files;
    if (!fs::exists(root)) {
        return files;
    }

    for (const auto& entry : fs::recursive_directory_iterator(root)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path());
        }
    }
    return files;
}

void requireNoFormInternalLeakage(const fs::path& root) {
    const std::string forbidden_include =
        std::string("forms/") + "local_spherical_3dof" + "/internal/";
    const std::string allowed_form_path =
        "framework/include/gnc/forms/local_spherical_3dof/";
    const std::vector<fs::path> roots = {
        root / "framework/include/gnc",
        root / "src",
        root / "tests",
    };

    for (const auto& scan_root : roots) {
        for (const auto& file : collectFiles(scan_root)) {
            const std::string normalized_file = normalizePath(file);
            if (normalized_file.find(allowed_form_path) != std::string::npos) {
                continue;
            }

            const std::string text = readFile(file);
            test_support::require(
                text.find(forbidden_include) == std::string::npos,
                "Form-internal local_spherical_3dof header leaked outside the form package: " +
                    normalized_file);
        }
    }
}

void requireNoStaticRegistrationFallback(const fs::path& root) {
    const fs::path factory_path =
        root / "framework/include/gnc/core/component_factory.hpp";
    const std::string text = readFile(factory_path);

    test_support::require(
        text.find("ComponentType##_Registrar") == std::string::npos,
        "Static registrar fallback should not remain in component_factory.hpp.");
    test_support::require(
        text.find("registrar_instance") == std::string::npos,
        "Static registrar instance fallback should not remain in component_factory.hpp.");
    test_support::require(
        text.find("requires GNC_COMPONENT_REGISTRATION_FN") != std::string::npos,
        "component_factory.hpp should enforce explicit registration at compile time.");
}

} // namespace

int main() {
    try {
        const fs::path root = repoRoot();
        test_support::require(fs::exists(root / "CMakeLists.txt"),
                              "Could not locate repository root from __FILE__.");

        requireNoFormInternalLeakage(root);
        requireNoStaticRegistrationFallback(root);

        std::cout << "architecture guard checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
