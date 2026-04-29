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

void requireVehicleInputAndOutputBootstrapHooks(const fs::path& root) {
    const fs::path bootstrap_path =
        root / "framework/include/gnc/bootstrap/register_builtin_packages.hpp";
    const std::string text = readFile(bootstrap_path);

    test_support::require(
        text.find("registerVehicleInputPackages") != std::string::npos,
        "Builtin bootstrap should expose a formal vehicle.input registration hook.");
    test_support::require(
        text.find("registerVehicleOutputPackages") != std::string::npos,
        "Builtin bootstrap should expose a formal vehicle.output registration hook.");
}

void requireNoLegacyCoordinateServicePath(const fs::path& root) {
    const std::vector<fs::path> roots = {
        root / "framework/include/gnc",
        root / "src",
        root / "tests",
    };
    const std::string self_path = normalizePath(fs::path(__FILE__));

    for (const auto& scan_root : roots) {
        for (const auto& file : collectFiles(scan_root)) {
            const std::string normalized_file = normalizePath(file);
            if (normalized_file == self_path) {
                continue;
            }

            const std::string text = readFile(file);
            test_support::require(
                text.find("installBuiltinService") == std::string::npos,
                "Legacy service-installer helper should not remain in runtime/test code: " +
                    normalized_file);
            test_support::require(
                text.find("DeferredRegistryAction") == std::string::npos,
                "Deferred service action wiring should not remain in runtime/test code: " +
                    normalized_file);
            test_support::require(
                text.find("services/soviet_coord") == std::string::npos,
                "Legacy soviet_coord service headers should not remain in runtime/test code: " +
                    normalized_file);
        }
    }
}

void requireCoreDoesNotOwnConcreteServiceLogic(const fs::path& root) {
    const fs::path core_root = root / "framework/include/gnc/core";
    const std::vector<std::string> forbidden = {
        "coordinate_tree",
        "CoordinateTreeService",
        "CoordinateTreeBuilder",
        "CoordinateTreeSpecRegistry",
    };

    for (const auto& file : collectFiles(core_root)) {
        const std::string text = readFile(file);
        const std::string normalized_file = normalizePath(file);
        for (const auto& token : forbidden) {
            test_support::require(
                text.find(token) == std::string::npos,
                "Core service assembly should not depend on concrete service logic ('" +
                    token + "'): " + normalized_file);
        }
    }
}

void requireCoordinateProbeStaysOutOfBuiltins(const fs::path& root) {
    const fs::path builtin_bootstrap_path =
        root / "framework/include/gnc/bootstrap/register_builtin_packages.hpp";
    const std::string bootstrap_text = readFile(builtin_bootstrap_path);

    test_support::require(
        bootstrap_text.find("coordinate_probe") == std::string::npos,
        "Coordinate probe is a demo utility and should not be registered as a builtin.");
    test_support::require(
        bootstrap_text.find("CoordinateProbe") == std::string::npos,
        "CoordinateProbe should not be referenced by builtin package bootstrap.");
    test_support::require(
        !fs::exists(root / "framework/include/gnc/vehicle/process/components/coordinate_probe.hpp"),
        "Demo coordinate_probe component should not exist under framework vehicle.process.");
}

void requireTerminationIsComponentized(const fs::path& root) {
    test_support::require(
        !fs::exists(root / "framework/include/gnc/core/stop_condition_builder.hpp"),
        "StopConditionBuilder should not remain as a core assembly path.");

    const fs::path simulation_builder_path =
        root / "framework/include/gnc/core/simulation_builder.hpp";
    const std::string simulation_builder_text = readFile(simulation_builder_path);
    test_support::require(
        simulation_builder_text.find("StopConditionBuilder") == std::string::npos,
        "SimulationBuilder should not bind legacy stop_conditions through StopConditionBuilder.");
    test_support::require(
        simulation_builder_text.find("top-level 'termination' component") != std::string::npos,
        "SimulationBuilder should reject stop_conditions and direct users to termination components.");
}

void requireProjectPrivateIncludeContract(const fs::path& root) {
    const std::string cmake_text = readFile(root / "CMakeLists.txt");

    test_support::require(
        cmake_text.find("GNC_REGISTER_COMPONENT_TYPE") != std::string::npos,
        "CMake should reject non-registrable headers under user/<project>/components/.");
    test_support::require(
        cmake_text.find("${USER_PROJECT_DIR}/interfaces") != std::string::npos,
        "gnc_sim should include the active project's interfaces/ directory.");
    test_support::require(
        cmake_text.find("${USER_PROJECT_DIR}/common") != std::string::npos,
        "gnc_sim should include the active project's common/ directory.");
    test_support::require(
        cmake_text.find("${USER_PROJECT_DIR}/include") != std::string::npos,
        "gnc_sim should include the active project's include/ directory.");
}

void requireConfigParsingFacadeAndNewUserRoots(const fs::path& root) {
    const std::string config_text =
        readFile(root / "framework/include/gnc/core/config_manager.hpp");
    const std::string cmake_text = readFile(root / "CMakeLists.txt");
    const std::string runner_text = readFile(root / "src/runner.cpp");

    test_support::require(config_text.find("class IConfigParser") != std::string::npos,
                          "Config parsing should be hidden behind IConfigParser.");
    test_support::require(
        config_text.find("class BuiltinJsonConfigParser") != std::string::npos,
        "The builtin JSON parser should implement the parser facade.");
    test_support::require(
        config_text.find("class ConfigPreprocessor") != std::string::npos,
        "ConfigManager should use an explicit preprocessing layer.");
    test_support::require(config_text.find("class JsonParser") == std::string::npos,
                          "ConfigManager should not expose the old JsonParser type.");
    test_support::require(
        config_text.find("Config loaded from string cannot use filesystem $include") !=
            std::string::npos,
        "loadFromString should reject filesystem $include directives.");

    test_support::require(!fs::exists(root / "user/config"),
                          "Root-level user/config should not exist.");
    test_support::require(!fs::exists(root / "user/components"),
                          "Root-level user/components should not exist.");
    test_support::require(fs::exists(root / "user/data"),
                          "user/data should be the cross-project data root.");

    test_support::require(
        cmake_text.find("user/config/missions/default.json") == std::string::npos,
        "CMake should not keep a root-level user/config fallback mission.");
    test_support::require(
        cmake_text.find("GNC_ACTIVE_PROJECT_FROM_FILE") != std::string::npos,
        "CMake should read user/active_project as an explicit fallback.");
    test_support::require(
        cmake_text.find("GNC_ACTIVE_PROJECT cache value") != std::string::npos &&
            cmake_text.find("user/active_project value") != std::string::npos,
        "CMake should warn when the cache active project overrides user/active_project.");
    test_support::require(
        runner_text.find("No default mission is configured") != std::string::npos,
        "Runner should emit a clear error when no active project default mission exists.");
}

} // namespace

int main() {
    try {
        const fs::path root = repoRoot();
        test_support::require(fs::exists(root / "CMakeLists.txt"),
                              "Could not locate repository root from __FILE__.");

        requireNoFormInternalLeakage(root);
        requireNoStaticRegistrationFallback(root);
        requireVehicleInputAndOutputBootstrapHooks(root);
        requireNoLegacyCoordinateServicePath(root);
        requireCoreDoesNotOwnConcreteServiceLogic(root);
        requireCoordinateProbeStaysOutOfBuiltins(root);
        requireTerminationIsComponentized(root);
        requireProjectPrivateIncludeContract(root);
        requireConfigParsingFacadeAndNewUserRoots(root);

        std::cout << "architecture guard checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
