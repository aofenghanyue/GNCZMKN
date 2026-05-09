#include "test_support.hpp"

#include "gnc/core/simulation_builder.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/simulator.hpp"
#include "gnc/forms/cartesian_3dof/components/point_mass.hpp"
#include "gnc/interfaces/i_summary_observer.hpp"
#include "gnc/vehicle/output/interfaces/i_continuous_mass.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fs = std::filesystem;

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::vector<std::vector<std::string>> readCsv(const fs::path& path) {
    std::ifstream file(path);
    test_support::require(file.is_open(),
                          "Expected CSV was not created: " + path.generic_string());

    std::vector<std::vector<std::string>> rows;
    std::string line;
    while (std::getline(file, line)) {
        rows.push_back(splitCsvLine(line));
    }
    return rows;
}

std::string readText(const fs::path& path) {
    std::ifstream file(path);
    test_support::require(file.is_open(),
                          "Expected text file was not created: " + path.generic_string());

    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

size_t columnIndex(const std::vector<std::string>& header,
                   const std::string& name) {
    for (size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) {
            return i;
        }
    }
    throw std::runtime_error("CSV header missing column: " + name);
}

double csvNumber(const std::vector<std::string>& row, size_t index) {
    return std::stod(row.at(index));
}

std::string cartesianMission(const std::string& output_dir,
                             const std::string& session_name,
                             double duration,
                             double stop_threshold,
                             bool include_stop) {
    std::ostringstream json;
    json << R"json(
{
  "simulation": {
    "dt": 0.5,
    "duration": )json"
         << duration << R"json(,
    "integrator": "rk4"
  },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.cartesian_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "initial_position": [0.0, 0.0, 1000.0],
              "initial_velocity": [0.0, 0.0, 10.0]
            }
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "test_fixture.cartesian_3dof.acceleration_input",
            "name": "interaction",
            "config": {
              "acceleration_mps2": [0.0, 0.0, -2.0]
            }
          }
        ]
      }
    }
  ],
  "outputs": {
    "directory": ")json"
         << output_dir << R"json(",
    "session_name": ")json"
         << session_name << R"json(",
    "record": {
      "vehicle.dynamics": "all"
    }
  })json";

    if (include_stop) {
        json << R"json(,
  "termination": {
    "type": "termination.component_field_below",
    "name": "termination",
    "config": {
      "component": "vehicle.dynamics",
      "field": "altitude",
      "value": )json"
             << stop_threshold << R"json(,
      "description": "stop at t0"
    }
  })json";
    }

    json << "\n}\n";
    return json.str();
}

std::string localSphericalMission(const std::string& output_dir,
                                  const std::string& session_name) {
    const char* dynamics_component = R"json(
          {
            "type": "form.local_spherical_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "launch_azimuth_rad": 1.5707963267948966,
              "initial_state": {
                "longitude_rad": 1.9198621771937625,
                "latitude_rad": 0.5235987755982988,
                "altitude_m": 60000.0,
                "speed_mps": 3200.0,
                "flight_path_angle_rad": -0.1047197551196598,
                "heading_angle_rad": -1.5707963267948966
              }
            }
          })json";

    std::ostringstream json;
    json << R"json(
{
  "simulation": {
    "dt": 0.1,
    "duration": 0.1,
    "integrator": "rk4"
  },
  "environment": {
    "components": [
      {
        "type": "environment.spherical_earth",
        "name": "earth",
        "config": {}
      },
      {
        "type": "environment.standard_atmosphere",
        "name": "atmosphere",
        "config": {}
      }
    ]
  },
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
)json"
         << dynamics_component << R"json(
        ]
      },
      "common": [],
      "input": [
        { "type": "vehicle.input.air_data_3dof.ideal", "name": "air_data", "config": {} }
      ],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "test_fixture.local_spherical_3dof.acceleration_input",
            "name": "interaction",
            "config": {
              "local_acceleration_nue_mps2": [0.0, -9.80665, 0.0]
            }
          }
        ]
      }
    }
  ],
  "outputs": {
    "directory": ")json"
         << output_dir << R"json(",
    "session_name": ")json"
         << session_name << R"json(",
    "record": {
      "vehicle.dynamics": "all",
      "vehicle.air_data": "all"
    }
  }
}
)json";
    return json.str();
}

class LinearMass final : public gnc::core::ComponentBase,
                         public gnc::interfaces::IContinuousSystem,
                         public gnc::vehicle::output::IContinuousMass {
public:
    LinearMass() : ComponentBase("LinearMass") {
        mass_index_ = layout_.addVariable("mass_kg");
        state_ = Eigen::VectorXd::Constant(1, 10.0);
        initial_state_ = state_;
    }

    void update(double) override {}

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double,
                            const Eigen::VectorXd&,
                            Eigen::VectorXd& derivative) const override {
        derivative = Eigen::VectorXd::Constant(1, -2.0);
    }

    const Eigen::VectorXd& getState() const override { return state_; }
    void setState(const Eigen::VectorXd& state) override { state_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_; }

    double getMassKg() const override { return state_[mass_index_]; }
    double getMassRateKgPerSec() const override { return -2.0; }

private:
    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    int mass_index_ = -1;
};

class MassDrivenPosition final : public gnc::core::ComponentBase,
                                 public gnc::interfaces::IContinuousSystem {
public:
    MassDrivenPosition() : ComponentBase("MassDrivenPosition") {
        position_index_ = layout_.addVariable("position");
        state_ = Eigen::VectorXd::Zero(1);
        initial_state_ = state_;
    }

    void setMass(gnc::vehicle::output::IContinuousMass* mass) { mass_ = mass; }
    void update(double) override {}

    const gnc::core::StateLayout& getStateLayout() const override { return layout_; }

    void computeDerivatives(double,
                            const Eigen::VectorXd&,
                            Eigen::VectorXd& derivative) const override {
        derivative = Eigen::VectorXd::Constant(1, mass_ ? mass_->getMassKg() : 0.0);
    }

    const Eigen::VectorXd& getState() const override { return state_; }
    void setState(const Eigen::VectorXd& state) override { state_ = state; }
    Eigen::VectorXd getInitialState() const override { return initial_state_; }

    double position() const { return state_[position_index_]; }

private:
    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_;
    Eigen::VectorXd initial_state_;
    gnc::vehicle::output::IContinuousMass* mass_ = nullptr;
    int position_index_ = -1;
};

class OrderProbe final : public gnc::core::ComponentBase {
public:
    OrderProbe(std::string id, std::vector<std::string>* log)
        : ComponentBase("OrderProbe"), id_(std::move(id)), log_(log) {}

    void update(double) override {
        if (getSimStep() == 0 && log_) {
            log_->push_back(id_);
        }
    }

private:
    std::string id_;
    std::vector<std::string>* log_ = nullptr;
};

std::vector<std::string>* g_json_order_log = nullptr;

class JsonOrderProbe final : public gnc::core::ComponentBase {
public:
    JsonOrderProbe() : ComponentBase("JsonOrderProbe") {}

    void update(double) override {
        if (getSimStep() == 0 && g_json_order_log) {
            g_json_order_log->push_back(getName());
        }
    }
};

class SummaryProbe final : public gnc::core::ComponentBase,
                           public gnc::interfaces::ISummaryObserver {
public:
    SummaryProbe() : ComponentBase("SummaryProbe") {}

    void update(double) override { ++updates_; }

    void writeSummary(std::ostream& out) const override {
        out << "  updates: " << updates_ << "\n";
    }

private:
    int updates_ = 0;
};

void registerPublishSemanticsTestTypes() {
    auto& factory = gnc::core::ComponentFactory::instance();
    if (!factory.hasType("test.publish_order_probe")) {
        factory.registerType<JsonOrderProbe>(
            "test.publish_order_probe",
            gnc::core::ComponentCategory::Project,
            __FILE__,
            gnc::core::ComponentPackageRole::VehicleProcess,
            gnc::core::ExecutionStage::VehicleProcess);
    }
    if (!factory.hasType("test.summary_probe")) {
        factory.registerType<SummaryProbe,
                             gnc::interfaces::ISummaryObserver>(
            "test.summary_probe",
            gnc::core::ComponentCategory::Project,
            __FILE__,
            gnc::core::ComponentPackageRole::Summary,
            gnc::core::ExecutionStage::Summary);
    }
}

} // namespace

int main() {
    const fs::path root = fs::path("user/outputs/test_publish_semantics");
    std::error_code cleanup_error;
    fs::remove_all(root, cleanup_error);

    try {
        test_support::registerBuiltinComponentTypes();
        registerPublishSemanticsTestTypes();

        {
            gnc::core::SimulationBuilder builder;
            test_support::require(
                builder.loadConfigString(cartesianMission(
                    (root / "timing").generic_string(), "publish_timing", 1.0, 0.0, false)),
                "Publish timing mission JSON could not be parsed.");
            auto& simulator = builder.build();
            simulator.run();

            const auto rows = readCsv(root / "timing" / "publish_timing.csv");
            test_support::require(rows.size() == 4,
                                  "CSV should contain header plus t0, t1, and t2 rows.");
            const auto& header = rows.front();
            const auto time_col = columnIndex(header, "time");
            const auto z_col = columnIndex(header, "vehicle.dynamics.position.z");
            const auto vz_col = columnIndex(header, "vehicle.dynamics.velocity.z");

            test_support::requireNear(csvNumber(rows[1], time_col), 0.0, 1e-12,
                                      "First CSV row must be t0.");
            test_support::requireNear(csvNumber(rows[1], z_col), 1000.0, 1e-9,
                                      "t0 altitude was not recorded at the published state.");
            test_support::requireNear(csvNumber(rows[2], time_col), 0.5, 1e-12,
                                      "Second CSV row must be t1.");
            test_support::requireNear(csvNumber(rows[2], z_col), 1004.75, 1e-9,
                                      "t1 altitude does not match constant-acceleration motion.");
            test_support::requireNear(csvNumber(rows[2], vz_col), 9.0, 1e-9,
                                      "t1 velocity does not match constant-acceleration motion.");
            test_support::requireNear(csvNumber(rows[3], time_col), 1.0, 1e-12,
                                      "Third CSV row must be t2.");
            test_support::requireNear(csvNumber(rows[3], z_col), 1009.0, 1e-9,
                                      "t2 altitude does not match constant-acceleration motion.");
        }

        {
            gnc::core::SimulationBuilder builder;
            test_support::require(
                builder.loadConfigString(localSphericalMission(
                    (root / "local_spherical_publish").generic_string(),
                    "local_spherical_publish")),
                "Local-spherical publish mission JSON could not be parsed.");
            auto& simulator = builder.build();
            simulator.run();

            const auto rows = readCsv(root / "local_spherical_publish" /
                                      "local_spherical_publish.csv");
            test_support::require(rows.size() == 3,
                                  "Local-spherical CSV should contain header plus t0 and t1.");
            const auto& header = rows.front();
            const auto dynamics_alt_col =
                columnIndex(header, "vehicle.dynamics.altitude_m");
            const auto air_data_alt_col =
                columnIndex(header, "vehicle.air_data.altitude_m");
            const auto air_data_mach_col =
                columnIndex(header, "vehicle.air_data.mach_number");
            test_support::requireNear(
                csvNumber(rows[2], air_data_alt_col),
                csvNumber(rows[2], dynamics_alt_col),
                1e-6,
                "Ideal air-data should observe the published local-spherical truth.");
            test_support::require(csvNumber(rows[1], air_data_mach_col) > 5.0,
                                  "Ideal air-data Mach number should reflect the initial state.");
        }

        {
            gnc::core::SimulationBuilder builder;
            test_support::require(
                builder.loadConfigString(cartesianMission(
                    (root / "stop_t0").generic_string(), "stop_t0", 1.0, 1001.0, true)),
                "t0 stop mission JSON could not be parsed.");
            auto& simulator = builder.build();
            simulator.run();

            test_support::require(simulator.getTerminationReason() == "stop at t0",
                                  "t0 stop condition did not terminate the mission.");
            const auto rows = readCsv(root / "stop_t0" / "stop_t0.csv");
            test_support::require(rows.size() == 2,
                                  "t0 stop should leave exactly one data row.");
            const auto time_col = columnIndex(rows.front(), "time");
            test_support::requireNear(csvNumber(rows[1], time_col), 0.0, 1e-12,
                                      "t0 stop row must be recorded at time zero.");
        }

        {
            const auto output_dir = root / "summary_observer";
            std::ostringstream mission;
            mission << R"json(
{
  "simulation": { "dt": 0.5, "duration": 0.5, "integrator": "rk4" },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.cartesian_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "initial_position": [0.0, 0.0, 1000.0],
              "initial_velocity": [0.0, 0.0, 0.0]
            }
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "test_fixture.cartesian_3dof.acceleration_input",
            "name": "interaction",
            "config": {
              "acceleration_mps2": [0.0, 0.0, 0.0]
            }
          }
        ]
      }
    }
  ],
  "outputs": {
    "directory": ")json"
                    << output_dir.generic_string() << R"json(",
    "session_name": "summary_observer",
    "record": {
      "vehicle.dynamics": "all"
    }
  },
  "summary": {
    "type": "test.summary_probe",
    "name": "summary",
    "config": {}
  }
}
)json";

            gnc::core::SimulationBuilder builder;
            test_support::require(builder.loadConfigString(mission.str()),
                                  "Summary observer mission JSON could not be parsed.");
            builder.build().run();

            const std::string summary_text = readText(output_dir / "summary.txt");
            test_support::require(
                summary_text.find("--- Project Summary ---") != std::string::npos,
                "Simulation summary did not include project summary section.");
            test_support::require(
                summary_text.find("[mission.summary]") != std::string::npos,
                "Simulation summary did not identify the mission summary component.");
            test_support::require(
                summary_text.find("updates: 2") != std::string::npos,
                "Summary observer did not write its accumulated metrics.");
        }

        {
            gnc::core::SimulationBuilder builder;
            test_support::require(
                builder.loadConfigString(cartesianMission(
                    (root / "callbacks").generic_string(), "callbacks", 0.5, 0.0, false)),
                "Callback mission JSON could not be parsed.");
            auto& simulator = builder.build();
            auto* dynamics = simulator.getRegistry().get<
                gnc::forms::cartesian_3dof::PointMass>("vehicle.dynamics");
            test_support::require(dynamics != nullptr,
                                  "Callback test did not build vehicle.dynamics.");

            bool before_ok = false;
            bool after_ok = false;
            simulator.onBeforeStep([&](int step, double time, double) {
                if (step == 0) {
                    before_ok = std::abs(time) < 1e-12 &&
                                std::abs(dynamics->getCartesian3DoFTruth().sample_time_s) <
                                    1e-12 &&
                                std::abs(dynamics->getAltitude() - 1000.0) < 1e-9;
                }
            });
            simulator.onAfterStep([&](int step, double time, double) {
                if (step == 0) {
                    after_ok = std::abs(time - 0.5) < 1e-12 &&
                               std::abs(dynamics->getAltitude() - 1004.75) < 1e-9 &&
                               std::abs(dynamics->getCartesian3DoFTruth().sample_time_s) <
                                   1e-12;
                }
            });
            simulator.run();
            test_support::require(before_ok,
                                  "before_step did not see the t_k published state.");
            test_support::require(after_ok,
                                  "after_step should run after integration and before next publish.");
        }

        {
            gnc::core::Simulator simulator;
            simulator.configure(gnc::core::SimulatorConfig{1.0, 1.0});

            auto mass = std::make_unique<LinearMass>();
            auto* mass_ptr = mass.get();
            simulator.getRegistry().add<LinearMass,
                                        gnc::interfaces::IContinuousSystem,
                                        gnc::vehicle::output::IContinuousMass>(
                "vehicle.mass", std::move(mass));
            simulator.addComponentToStage(mass_ptr,
                                          gnc::core::ExecutionStage::VehicleOutput);

            auto position = std::make_unique<MassDrivenPosition>();
            auto* position_ptr = position.get();
            position_ptr->setMass(mass_ptr);
            simulator.getRegistry().add<MassDrivenPosition,
                                        gnc::interfaces::IContinuousSystem>(
                "vehicle.position", std::move(position));
            simulator.addComponentToStage(position_ptr, gnc::core::ExecutionStage::Form);

            simulator.run();

            test_support::requireNear(mass_ptr->getMassKg(), 8.0, 1e-9,
                                      "Mass component did not integrate to t1.");
            test_support::requireNear(
                position_ptr->position(),
                10.0,
                1e-9,
                "Continuous systems should read t_k state and commit t_{k+1} together.");
        }

        {
            std::vector<std::string> order;
            gnc::core::Simulator simulator;
            simulator.configure(gnc::core::SimulatorConfig{1.0, 0.0});

            auto first = std::make_unique<OrderProbe>("first", &order);
            auto* first_ptr = first.get();
            simulator.getRegistry().add<OrderProbe>("first", std::move(first));
            simulator.addComponentToStage(first_ptr,
                                          gnc::core::ExecutionStage::VehicleProcess);

            auto second = std::make_unique<OrderProbe>("second", &order);
            auto* second_ptr = second.get();
            simulator.getRegistry().add<OrderProbe>("second", std::move(second));
            simulator.addComponentToStage(second_ptr,
                                          gnc::core::ExecutionStage::VehicleProcess);

            simulator.run();
            test_support::require(order.size() == 2 && order[0] == "first" &&
                                      order[1] == "second",
                                  "Equal priority should preserve registration/config order.");
        }

        {
            std::vector<std::string> order;
            g_json_order_log = &order;
            const char* mission = R"json(
{
  "simulation": { "dt": 1.0, "duration": 0.0, "integrator": "rk4" },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.cartesian_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "initial_position": [0.0, 0.0, 0.0],
              "initial_velocity": [0.0, 0.0, 0.0]
            }
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [
        {
          "type": "test.publish_order_probe",
          "name": "guidance",
          "priority": 20,
          "config": {}
        },
        {
          "type": "test.publish_order_probe",
          "name": "navigation",
          "priority": 0,
          "config": {}
        },
        {
          "type": "test.publish_order_probe",
          "name": "observer",
          "priority": 0,
          "config": {}
        },
        {
          "type": "test.publish_order_probe",
          "name": "flight_view",
          "priority": 10,
          "config": {}
        }
      ],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "test_fixture.cartesian_3dof.acceleration_input",
            "name": "interaction",
            "config": {
              "acceleration_mps2": [0.0, 0.0, 0.0]
            }
          }
        ]
      }
    }
  ],
  "outputs": { "enabled": false }
}
)json";

            gnc::core::SimulationBuilder builder;
            test_support::require(builder.loadConfigString(mission),
                                  "JSON priority mission could not be parsed.");
            builder.build().run();
            g_json_order_log = nullptr;
            test_support::require(order.size() == 4 && order[0] == "vehicle.navigation" &&
                                      order[1] == "vehicle.observer" &&
                                      order[2] == "vehicle.flight_view" &&
                                      order[3] == "vehicle.guidance",
                                  "Navigation, flight_view, and guidance should be orderable inside one stage by JSON priority while equal priorities keep JSON order.");
        }

        {
            std::vector<std::string> order;
            gnc::core::Simulator simulator;
            simulator.configure(gnc::core::SimulatorConfig{1.0, 0.0});

            auto late = std::make_unique<OrderProbe>("late", &order);
            auto* late_ptr = late.get();
            late_ptr->setPriorityInternal_(20);
            simulator.getRegistry().add<OrderProbe>("late", std::move(late));
            simulator.addComponentToStage(late_ptr,
                                          gnc::core::ExecutionStage::VehicleProcess);

            auto early = std::make_unique<OrderProbe>("early", &order);
            auto* early_ptr = early.get();
            early_ptr->setPriorityInternal_(-10);
            simulator.getRegistry().add<OrderProbe>("early", std::move(early));
            simulator.addComponentToStage(early_ptr,
                                          gnc::core::ExecutionStage::VehicleProcess);

            simulator.run();
            test_support::require(order.size() == 2 && order[0] == "early" &&
                                      order[1] == "late",
                                  "Priority should adjust order within the same execution stage.");
        }

        fs::remove_all(root, cleanup_error);
        std::cout << "publish semantics checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        fs::remove_all(root, cleanup_error);
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
