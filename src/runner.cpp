#include "gnc/common/logger.hpp"
#include "auto_registered_components.hpp"
#include "gnc/bootstrap/register_builtin_packages.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/simulation_builder.hpp"
#include "gnc/runset/runset_runner.hpp"
#include "active_project_config.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

using namespace gnc::core;

namespace {

namespace fs = std::filesystem;

enum class RunnerMode {
    Run,
    RunSet,
    ListComponents,
    Help
};

struct RunnerOptions {
    RunnerMode mode = RunnerMode::Run;
    bool list_verbose = false;
    bool use_default_config = true;
    std::string requested_config;
    std::string requested_runset;
    int jobs = 1;
    bool jobs_specified = false;
};

struct ConfigResolution {
    std::string resolved_path;
    fs::path anchor_root;
};

std::string normalizePath(const fs::path& path) {
    return path.lexically_normal().generic_string();
}

void appendUniquePath(std::vector<fs::path>& paths, const fs::path& candidate) {
    if (candidate.empty()) {
        return;
    }

    const std::string normalized = normalizePath(candidate);
    for (const auto& existing : paths) {
        if (normalizePath(existing) == normalized) {
            return;
        }
    }
    paths.push_back(candidate);
}

std::vector<fs::path> collectSearchRoots(const char* program_name) {
    std::vector<fs::path> roots;
    std::error_code ec;

    appendUniquePath(roots, fs::current_path(ec));

    ec.clear();
    const fs::path program_path = fs::absolute(fs::path(program_name ? program_name : ""), ec);
    if (!ec && !program_path.empty()) {
        appendUniquePath(roots, program_path.parent_path());
    }

    return roots;
}

std::string describeCandidatePaths(const std::vector<fs::path>& candidates) {
    if (candidates.empty()) {
        return "(none)";
    }

    std::string result;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (i > 0) {
            result += ", ";
        }
        result += normalizePath(candidates[i]);
    }
    return result;
}

std::string joinStrings(const std::vector<std::string>& values) {
    if (values.empty()) {
        return "(none)";
    }

    std::string result;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            result += ", ";
        }
        result += values[i];
    }
    return result;
}

bool resolveConfigPath(const std::string& requested_path,
                       const char* program_name,
                       ConfigResolution& resolution,
                       std::vector<fs::path>& searched_paths) {
    searched_paths.clear();
    resolution = {};

    const fs::path config_path(requested_path);
    std::error_code ec;

    if (config_path.is_absolute()) {
        appendUniquePath(searched_paths, config_path);
        if (!fs::exists(config_path, ec) || ec) {
            return false;
        }

        const fs::path absolute_candidate = fs::absolute(config_path, ec);
        resolution.resolved_path = ec ? normalizePath(config_path) : normalizePath(absolute_candidate);
        return true;
    }

    for (fs::path root : collectSearchRoots(program_name)) {
        root = root.lexically_normal();
        while (!root.empty()) {
            const fs::path candidate = root / config_path;
            appendUniquePath(searched_paths, candidate);

            ec.clear();
            if (fs::exists(candidate, ec) && !ec) {
                ec.clear();
                const fs::path absolute_candidate = fs::absolute(candidate, ec);
                resolution.resolved_path = ec ? normalizePath(candidate) : normalizePath(absolute_candidate);
                resolution.anchor_root = root;
                return true;
            }

            const fs::path parent = root.parent_path();
            if (parent == root) {
                break;
            }
            root = parent;
        }
    }

    return false;
}

bool setMode(RunnerOptions& options, RunnerMode mode, std::string& error) {
    if (options.mode != RunnerMode::Run && options.mode != mode) {
        error = "Only one CLI action can be used at a time.";
        return false;
    }
    options.mode = mode;
    return true;
}

bool parseArgs(int argc, char* argv[], RunnerOptions& options, std::string& error) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            return setMode(options, RunnerMode::Help, error);
        }

        if (arg == "--list-components") {
            options.list_verbose = false;
            if (!setMode(options, RunnerMode::ListComponents, error)) {
                return false;
            }
            continue;
        }

        if (arg == "--list-components-verbose") {
            options.list_verbose = true;
            if (!setMode(options, RunnerMode::ListComponents, error)) {
                return false;
            }
            continue;
        }

        if (arg == "--config") {
            if (i + 1 >= argc) {
                error = "Missing path after --config.";
                return false;
            }
            if (options.mode != RunnerMode::Run) {
                error = "The --config option cannot be combined with other CLI actions.";
                return false;
            }
            if (!options.requested_config.empty()) {
                error = "Config file was specified more than once.";
                return false;
            }
            options.requested_config = argv[++i];
            options.use_default_config = false;
            continue;
        }

        if (arg == "--runset") {
            if (i + 1 >= argc) {
                error = "Missing path after --runset.";
                return false;
            }
            if (!options.requested_config.empty()) {
                error = "The --runset option cannot be combined with a config path.";
                return false;
            }
            if (!setMode(options, RunnerMode::RunSet, error)) {
                return false;
            }
            options.requested_runset = argv[++i];
            options.use_default_config = false;
            continue;
        }

        if (arg == "--jobs") {
            if (i + 1 >= argc) {
                error = "Missing value after --jobs.";
                return false;
            }
            const std::string value = argv[++i];
            options.jobs_specified = true;
            if (value == "auto") {
                options.jobs = 0;
            } else {
                try {
                    size_t consumed = 0;
                    options.jobs = std::stoi(value, &consumed);
                    if (consumed != value.size() || options.jobs < 1) {
                        error = "--jobs must be a positive integer or auto.";
                        return false;
                    }
                } catch (const std::exception&) {
                    error = "--jobs must be a positive integer or auto.";
                    return false;
                }
            }
            continue;
        }

        if (!arg.empty() && arg[0] == '-') {
            error = "Unknown option: " + arg;
            return false;
        }

        if (options.mode != RunnerMode::Run) {
            error = "A config path cannot be combined with other CLI actions.";
            return false;
        }
        if (!options.requested_config.empty()) {
            error = "Config file was specified more than once.";
            return false;
        }
        options.requested_config = arg;
        options.use_default_config = false;
    }

    if (options.jobs_specified && options.mode != RunnerMode::RunSet) {
        error = "--jobs can only be used with --runset.";
        return false;
    }
    if (options.mode == RunnerMode::RunSet && options.requested_runset.empty()) {
        error = "RunSet mode requires --runset <runset.json>.";
        return false;
    }
    if (options.mode == RunnerMode::RunSet && options.jobs == 0) {
        error = "--jobs auto is reserved for the multiprocess phase.";
        return false;
    }
    if (options.mode == RunnerMode::RunSet && options.jobs > 1) {
        error = "--jobs > 1 is reserved for the multiprocess phase.";
        return false;
    }

    return true;
}

void printUsage(const char* program_name) {
    std::cout << "GNC Simulation Framework v2.0\n"
              << "Usage:\n"
              << "  " << program_name << "\n"
              << "  " << program_name << " <config.json>\n"
              << "  " << program_name << " --config <config.json>\n"
              << "  " << program_name << " --runset <runset.json>\n"
              << "  " << program_name << " --runset <runset.json> --jobs 1\n"
              << "  " << program_name << " --list-components\n"
              << "  " << program_name << " --list-components-verbose\n"
              << "  " << program_name << " --help\n";
    if (std::string_view(gnc::build::kDefaultConfigRelative).empty()) {
        std::cout
            << "Default mission lookup: none; set user/active_project or pass --config.\n";
    } else {
        std::cout << "Default mission lookup: "
                  << gnc::build::kDefaultConfigRelative << "\n";
    }
    if (std::string_view(gnc::build::kActiveProjectName).size() > 0) {
        std::cout << "Active project: " << gnc::build::kActiveProjectName << "\n";
    }
    std::cout
              << "  The runner searches upward from the current directory and the executable directory.\n";
}

void listComponents(bool verbose) {
    const auto infos = ComponentFactory::instance().getRegisteredTypeInfos();
    std::vector<std::string> builtin_types;
    std::vector<std::string> project_types;

    for (const auto& info : infos) {
        if (info.category == ComponentCategory::Builtin) {
            builtin_types.push_back(info.type_name);
        } else {
            project_types.push_back(info.type_name);
        }
    }

    std::cout << "Registered component types (" << infos.size() << "):\n";

    std::cout << "  Builtin components (" << builtin_types.size() << "):\n";
    for (const auto& info : infos) {
        if (info.category != ComponentCategory::Builtin) {
            continue;
        }
        std::cout << "    - " << info.type_name;
        if (verbose) {
            std::cout << " [interfaces: " << joinStrings(info.interface_names);
            std::cout << "; role: " << toString(info.package_role);
            std::cout << "; stage: " << toString(info.execution_stage);
            if (!info.form_family.empty()) {
                std::cout << "; form-family: " << info.form_family;
            }
            if (!info.registration_origin.empty()) {
                std::cout << "; origin: " << info.registration_origin;
            }
            std::cout << "]";
        }
        std::cout << "\n";
    }

    std::cout << "  Project/example components (" << project_types.size() << "):\n";
    for (const auto& info : infos) {
        if (info.category != ComponentCategory::Project) {
            continue;
        }
        std::cout << "    - " << info.type_name;
        if (verbose) {
            std::cout << " [interfaces: " << joinStrings(info.interface_names);
            std::cout << "; role: " << toString(info.package_role);
            std::cout << "; stage: " << toString(info.execution_stage);
            if (!info.form_family.empty()) {
                std::cout << "; form-family: " << info.form_family;
            }
            if (!info.registration_origin.empty()) {
                std::cout << "; origin: " << info.registration_origin;
            }
            std::cout << "]";
        }
        std::cout << "\n";
    }
}

}

int main(int argc, char* argv[]) {
    auto& factory = ComponentFactory::instance();
    gnc::bootstrap::registerBuiltinPackages(factory);
    gnc::build::registerAutoRegisteredProjectComponents(factory);

    RunnerOptions options;
    std::string parse_error;
    if (!parseArgs(argc, argv, options, parse_error)) {
        LOG_ERROR("{}", parse_error);
        printUsage(argv[0]);
        return 1;
    }

    if (options.mode == RunnerMode::Help) {
        printUsage(argv[0]);
        return 0;
    }
    if (options.mode == RunnerMode::ListComponents) {
        listComponents(options.list_verbose);
        return 0;
    }
    if (options.mode == RunnerMode::RunSet) {
        try {
            gnc::runset::RunSetRunner runner;
            runner.runSerial(options.requested_runset);
            LOG_INFO("=== RunSet Completed ===");
            return 0;
        } catch (const std::exception& e) {
            LOG_ERROR("RunSet failed: {}", e.what());
            return 1;
        }
    }

    const std::string requested_config =
        options.use_default_config ? gnc::build::kDefaultConfigRelative : options.requested_config;
    if (options.use_default_config &&
        std::string_view(gnc::build::kDefaultConfigRelative).empty()) {
        LOG_ERROR("No default mission is configured. Set user/active_project, configure with -DGNC_ACTIVE_PROJECT=<project>, or pass --config <path>.");
        return 1;
    }
    ConfigResolution config_resolution;
    std::vector<fs::path> searched_paths;
    if (!resolveConfigPath(requested_config, argv[0], config_resolution, searched_paths)) {
        if (options.use_default_config) {
            LOG_ERROR("No default mission could be found. Looked for '{}' by searching upward from the current directory and executable directory. Checked: {}. Pass --config <path> to run an explicit mission.",
                      requested_config,
                      describeCandidatePaths(searched_paths));
        } else {
            LOG_ERROR("Config file '{}' was not found. Checked: {}. You can pass an absolute path, a path relative to the current directory, or a repo-relative path such as --config user/example_02_atmospheric_3dof/config/mission.json.",
                      requested_config,
                      describeCandidatePaths(searched_paths));
        }
        return 1;
    }

    if (!config_resolution.anchor_root.empty()) {
        std::error_code ec;
        fs::current_path(config_resolution.anchor_root, ec);
        if (ec) {
            LOG_WARNING("Resolved mission root '{}' was found, but the process working directory could not be updated: {}",
                        normalizePath(config_resolution.anchor_root),
                        ec.message());
        }
    }

    const std::string& config_file = config_resolution.resolved_path;

    LOG_INFO("=== GNC Simulation Framework v2.0 ===");
    LOG_INFO("Config: {}", config_file);

    try {
        SimulationBuilder builder;
        if (!builder.loadConfig(config_file)) {
            LOG_ERROR("Failed to load config file '{}' in runner. Please check the file path and JSON syntax.", config_file);
            return 1;
        }

        auto& simulator = builder.build();
        simulator.run();

        LOG_INFO("=== Simulation Completed ===");
        return 0;
    } catch (const std::exception& e) {
        LOG_ERROR("Simulation failed in runner: {}. Please review the diagnostics above and fix the reported issue.", e.what());
        return 1;
    }
}
