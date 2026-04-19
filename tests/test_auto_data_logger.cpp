#include "test_support.hpp"

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_registry.hpp"
#include "gnc/infrastructure/auto_data_logger.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace {

namespace fs = std::filesystem;

class ObservableProbe final : public gnc::core::ComponentBase,
                              public gnc::interfaces::IObservable {
public:
    ObservableProbe(std::string class_name,
                    double altitude_m,
                    gnc::math::Vector3 velocity_mps)
        : ComponentBase(std::move(class_name)),
          altitude_m_(altitude_m),
          velocity_mps_(std::move(velocity_mps)) {}

    void update(double) override {}

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("altitude_m", [this]() { return altitude_m_; });
        builder.addVector3("velocity", [this]() -> const gnc::math::Vector3& {
            return velocity_mps_;
        });
        return builder.build();
    }

    void setDebugSnapshot(double residual, double iterations) {
        snapDebug("solver.residual", residual);
        snapDebug("solver.iterations", iterations);
    }

private:
    double altitude_m_ = 0.0;
    gnc::math::Vector3 velocity_mps_ = gnc::math::Vector3::Zero();
};

std::string readFile(const fs::path& path) {
    std::ifstream file(path);
    test_support::require(file.is_open(),
                          "Expected log file was not created: " + path.generic_string());
    return std::string(std::istreambuf_iterator<char>(file),
                       std::istreambuf_iterator<char>());
}

bool containsText(const std::string& text, const std::string& needle) {
    return text.find(needle) != std::string::npos;
}

gnc::core::ConfigNode makeExactExcludeConfig(const fs::path& output_dir) {
    using namespace test_support;
    return object({
        field("directory", string(output_dir.generic_string())),
        field("session_name", string("stable_fields")),
        field("record", object({field("missile.dynamics", string("all"))})),
        field("exclude", array({
            string("missile.dynamics.altitude_m"),
            string("*.velocity.z"),
        })),
    });
}

gnc::core::ConfigNode makeDebugAllConfig(const fs::path& output_dir) {
    using namespace test_support;
    return object({
        field("directory", string(output_dir.generic_string())),
        field("session_name", string("all_snapshots")),
        field("record", gnc::core::ConfigNode::makeArray()),
        field("debug_snapshots", boolean(true)),
    });
}

gnc::core::ConfigNode makeFilteredDebugConfig(const fs::path& output_dir) {
    using namespace test_support;
    return object({
        field("directory", string(output_dir.generic_string())),
        field("session_name", string("base_session")),
        field("record", gnc::core::ConfigNode::makeArray()),
        field("debug_snapshots",
              object({
                  field("components", array({string("missile.guidance")})),
                  field("session_name", string("filtered_snapshots")),
                  field("precision", number(4.0)),
                  field("flush_every_step", boolean(true)),
              })),
    });
}

} // namespace

int main() {
    const fs::path root = fs::path("user/outputs/test_auto_data_logger");
    std::error_code cleanup_error;
    fs::remove_all(root, cleanup_error);

    try {
        gnc::core::ComponentRegistry registry;

        auto dynamics = std::make_unique<ObservableProbe>(
            "ObservableProbe", 1200.0, gnc::math::Vector3(11.0, 12.0, 13.0));
        auto* dynamics_ptr = dynamics.get();
        registry.add<ObservableProbe, gnc::interfaces::IObservable>(
            "missile.dynamics", std::move(dynamics));

        auto guidance = std::make_unique<ObservableProbe>(
            "ObservableProbe", 900.0, gnc::math::Vector3(21.0, 22.0, 23.0));
        auto* guidance_ptr = guidance.get();
        registry.add<ObservableProbe, gnc::interfaces::IObservable>(
            "missile.guidance", std::move(guidance));

        {
            gnc::core::AutoDataLogger logger;
            test_support::require(
                logger.initialize(makeExactExcludeConfig(root / "exclude_rules"), registry),
                "AutoDataLogger failed to initialize exact/wildcard exclude test.");
            test_support::require(logger.getFieldCount() == 2,
                                  "Exclude rules should leave only velocity.x and velocity.y.");
            logger.recordStep(1.5);
            logger.stop();

            const std::string csv =
                readFile(root / "exclude_rules" / "stable_fields.csv");
            test_support::require(
                containsText(csv, "time,missile.dynamics.velocity.x,missile.dynamics.velocity.y"),
                "Stable-field CSV header did not keep the expected fields after excludes.");
            test_support::require(!containsText(csv, "altitude_m"),
                                  "Exact-field exclude no longer removes the full field name.");
            test_support::require(!containsText(csv, "velocity.z"),
                                  "Wildcard exclude no longer removes suffix-matched fields.");
        }

        dynamics_ptr->setDebugSnapshot(0.123456, 7.0);
        guidance_ptr->setDebugSnapshot(9.876543, 3.0);
        {
            gnc::core::AutoDataLogger logger;
            test_support::require(
                logger.initialize(makeDebugAllConfig(root / "debug_all"), registry),
                "AutoDataLogger failed to initialize debug snapshot bool config.");
            test_support::require(logger.isDebugSnapshotsEnabled(),
                                  "debug_snapshots: true should enable debug snapshot capture.");
            test_support::require(
                logger.getDebugSnapshotPath().find("all_snapshots_debug_snapshots.csv") !=
                    std::string::npos,
                "Boolean debug snapshot config should use the default session suffix.");
            logger.recordStep(2.0);
            logger.stop();

            const std::string csv =
                readFile(root / "debug_all" / "all_snapshots_debug_snapshots.csv");
            test_support::require(containsText(csv, "time,component,field,value"),
                                  "Debug snapshot CSV header is missing.");
            test_support::require(containsText(csv, "missile.dynamics,solver.residual"),
                                  "debug_snapshots: true should include the dynamics component.");
            test_support::require(containsText(csv, "missile.guidance,solver.residual"),
                                  "debug_snapshots: true should include the guidance component.");
        }

        dynamics_ptr->setDebugSnapshot(7.89012, 8.0);
        guidance_ptr->setDebugSnapshot(1.23456, 5.0);
        {
            gnc::core::AutoDataLogger logger;
            const fs::path filtered_file = root / "debug_filtered" / "filtered_snapshots.csv";

            test_support::require(
                logger.initialize(makeFilteredDebugConfig(root / "debug_filtered"), registry),
                "AutoDataLogger failed to initialize filtered debug snapshot config.");
            test_support::require(logger.isDebugSnapshotsEnabled(),
                                  "Object debug snapshot config should enable debug snapshots.");
            test_support::require(
                logger.getDebugSnapshotPath() == filtered_file.generic_string(),
                "Custom debug snapshot session_name should control the output filename.");

            logger.recordStep(3.0);
            std::error_code size_error;
            const auto visible_size = fs::file_size(filtered_file, size_error);
            test_support::require(!size_error && visible_size > 0,
                                  "flush_every_step should make the debug snapshot file visible before stop().");
            logger.stop();

            const std::string csv = readFile(filtered_file);
            test_support::require(
                containsText(csv, "missile.guidance,solver.residual,1.235"),
                "Filtered debug snapshot config should honor custom precision.");
            test_support::require(!containsText(csv, "missile.dynamics"),
                                  "Filtered debug snapshot config should honor the component filter.");
        }

        fs::remove_all(root, cleanup_error);
        std::cout << "auto data logger checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        fs::remove_all(root, cleanup_error);
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
