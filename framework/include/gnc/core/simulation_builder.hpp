#pragma once

#include "gnc/bootstrap/register_builtin_service_packages.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/integrators/euler_integrator.hpp"
#include "gnc/core/integrators/rk4_integrator.hpp"
#include "gnc/core/mission_assembler.hpp"
#include "gnc/core/service_package_registry.hpp"
#include "gnc/core/simulator.hpp"
#include "gnc/core/stop_condition_builder.hpp"
#include "gnc/core/validation_pipeline.hpp"

#include <algorithm>
#include <sstream>

namespace gnc::core {

class SimulationBuilder {
public:
    SimulationBuilder() {
        gnc::bootstrap::registerBuiltinServicePackages(service_packages_);
    }

    bool loadConfig(const std::string& filename) {
        return config_.loadFromFile(filename);
    }

    bool loadConfigString(const std::string& json) {
        return config_.loadFromString(json);
    }

    Simulator& build() {
        build_errors_.clear();
        build_warnings_.clear();
        simulator_.resetAssemblyState();

        MissionAssembler assembler(
            simulator_,
            global_services_,
            environment_,
            vehicles_,
            service_packages_,
            [this](const std::string& message) { addBuildError(message); },
            [this](const std::string& message) { addBuildWarning(message); });
        assembler.reset();

        const auto& simulation = config_.simulation();
        SimulatorConfig simulator_config;
        simulator_config.dt = simulation["dt"].asDouble(0.01);
        simulator_config.duration = simulation["duration"].asDouble(10.0);
        simulator_.configure(simulator_config);
        buildIntegrator();

        assembler.installGlobalServices(config_.globalServices());
        buildMissionArchitecture(assembler);
        assembler.finalizeServices();

        const auto validation = ValidationPipeline::run(
            simulator_.getRegistry(),
            assembler.getAssemblyDescriptors(),
            assembler.getSelectedFormFamily());
        for (const auto& error : validation.errors) {
            addBuildError(error);
        }
        for (const auto& warning : validation.warnings) {
            addBuildWarning(warning);
        }

        StopConditionBuilder stop_conditions(
            simulator_, [this](const std::string& message) { addBuildWarning(message); });
        stop_conditions.build(config_.stopConditions());

        if (!simulator_.initializeAutoDataLogger(config_.outputs())) {
            addBuildError("AutoDataLogger initialization failed during simulation build.");
        }

        if (!reportBuildDiagnostics()) {
            throw std::runtime_error("Simulation build failed with " +
                                     std::to_string(build_errors_.size()) +
                                     " error(s).");
        }

        logComponentInventory();
        return simulator_;
    }

    ConfigManager& getConfigManager() { return config_; }
    Simulator& getSimulator() { return simulator_; }
    ServiceContext& getGlobalServices() { return global_services_; }
    EnvironmentInstance& getEnvironment() { return environment_; }
    std::vector<VehicleInstance>& getVehicles() { return vehicles_; }
    const std::vector<std::string>& getBuildErrors() const { return build_errors_; }
    const std::vector<std::string>& getBuildWarnings() const { return build_warnings_; }

private:
    static std::string joinStrings(const std::vector<std::string>& values) {
        std::string result;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) {
                result += ", ";
            }
            result += values[i];
        }
        return result.empty() ? "(none)" : result;
    }

    void logComponentInventory() {
        std::vector<std::string> builtin_types;
        std::vector<std::string> project_types;

        for (const auto& name : simulator_.getRegistry().getComponentNames()) {
            auto* component = simulator_.getRegistry().get<ComponentBase>(name);
            if (!component) {
                continue;
            }

            if (component->getComponentCategory() == "builtin") {
                builtin_types.push_back(component->getTypeName());
            } else {
                project_types.push_back(component->getTypeName());
            }
        }

        auto unique_join = [](std::vector<std::string> values) {
            std::sort(values.begin(), values.end());
            values.erase(std::unique(values.begin(), values.end()), values.end());
            return joinStrings(values);
        };

        LOG_INFO("Mission component inventory: builtin types [{}], project types [{}]",
                 unique_join(std::move(builtin_types)),
                 unique_join(std::move(project_types)));
    }

    void buildMissionArchitecture(MissionAssembler& assembler) {
        const auto& root = config_.root();

        if (config_.hasEntities()) {
            addBuildError(
                "Legacy top-level 'entities[]' missions are no longer supported. "
                "Migrate the mission to the canonical top-level 'vehicles[]' layout.");
            return;
        }

        if (root.has("components") || root.has("services") || root.has("form") ||
            root.has("vehicle") || root.has("interaction")) {
            addBuildError(
                "Legacy root-level mission blocks ('components' / 'services' / "
                "'form' / 'vehicle' / 'interaction') are no longer supported. "
                "Use the canonical top-level 'vehicles[]' layout.");
            return;
        }

        if (!config_.hasModernMissionLayout()) {
            addBuildError(
                "Mission configuration must define a top-level 'vehicles' array.");
            return;
        }

        assembler.buildMission(root);
    }

    void buildIntegrator() {
        const std::string integrator_name =
            config_.simulation()["integrator"].asString("rk4");
        if (integrator_name == "rk4") {
            simulator_.setIntegrator(std::make_unique<RK4Integrator>());
            return;
        }
        if (integrator_name == "euler") {
            simulator_.setIntegrator(std::make_unique<EulerIntegrator>());
            return;
        }

        addBuildWarning("Unknown integrator '" + integrator_name +
                        "'. Falling back to RK4.");
        simulator_.setIntegrator(std::make_unique<RK4Integrator>());
    }

    void addBuildError(const std::string& message) {
        build_errors_.push_back(message);
    }

    void addBuildWarning(const std::string& message) {
        build_warnings_.push_back(message);
    }

    bool reportBuildDiagnostics() {
        if (build_errors_.empty() && build_warnings_.empty()) {
            return true;
        }

        std::ostringstream report;
        report << "\n=== Simulation Build Diagnostics ===\n";

        if (!build_errors_.empty()) {
            report << "[ERRORS] (" << build_errors_.size() << ")\n";
            for (size_t i = 0; i < build_errors_.size(); ++i) {
                report << "  " << (i + 1) << ". " << build_errors_[i] << "\n";
            }
        }

        if (!build_warnings_.empty()) {
            report << "[WARNINGS] (" << build_warnings_.size() << ")\n";
            for (size_t i = 0; i < build_warnings_.size(); ++i) {
                report << "  " << (i + 1) << ". " << build_warnings_[i] << "\n";
            }
        }

        report << "====================================";
        if (!build_errors_.empty()) {
            LOG_ERROR("{}", report.str());
            return false;
        }

        LOG_WARNING("{}", report.str());
        return true;
    }

    ConfigManager config_;
    Simulator simulator_;
    ServiceContext global_services_;
    EnvironmentInstance environment_;
    std::vector<VehicleInstance> vehicles_;
    ServicePackageRegistry service_packages_;
    std::vector<std::string> build_errors_;
    std::vector<std::string> build_warnings_;
};

} // namespace gnc::core
