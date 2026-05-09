#include "test_support.hpp"

#include "gnc/core/config_manager.hpp"
#include "gnc/core/simulation_builder.hpp"

#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace {

class MockConfigParser final : public gnc::core::IConfigParser {
public:
    explicit MockConfigParser(gnc::core::ConfigNode root) : root_(std::move(root)) {}

    gnc::core::ConfigNode parseFile(const std::string&) override { return root_; }
    gnc::core::ConfigNode parseString(const std::string&) override { return root_; }

private:
    gnc::core::ConfigNode root_;
};

gnc::core::ConfigNode makeMission() {
    using namespace test_support;
    return object({
        field("simulation",
              object({
                  field("dt", number(0.1)),
                  field("duration", number(0.1)),
              })),
        field("outputs", object({field("enabled", boolean(false))})),
        field("vehicles",
              array({
                  object({
                      field("id", string("vehicle")),
                      field("form",
                            object({
                                field("components",
                                      array({
                                          object({
                                              field("type",
                                                    string("form.cartesian_3dof.point_mass")),
                                              field("name", string("dynamics")),
                                              field("config",
                                                    object({
                                                        field("initial_position",
                                                              array({number(0.0),
                                                                     number(0.0),
                                                                     number(1000.0)})),
                                                        field("initial_velocity",
                                                              array({number(0.0),
                                                                     number(0.0),
                                                                     number(0.0)})),
                                                    })),
                                          }),
                                      })),
                            })),
                      field("common", array({})),
                      field("input", array({})),
                      field("process", array({})),
                      field("output", array({})),
                      field("interaction",
                            object({
                                field("components",
                                      array({
                                          object({
                                              field("type",
                                                    string("test_fixture.cartesian_3dof.acceleration_input")),
                                              field("name", string("interaction")),
                                              field("config",
                                                    object({
                                                        field("acceleration_mps2",
                                                              array({number(0.0),
                                                                     number(0.0),
                                                                     number(0.0)})),
                                                    })),
                                          }),
                                      })),
                            })),
                  }),
              })),
    });
}

} // namespace

int main() {
    try {
        test_support::registerBuiltinComponentTypes();

        auto parser = std::make_unique<MockConfigParser>(makeMission());
        gnc::core::SimulationBuilder builder(std::move(parser));
        test_support::require(builder.loadConfigString("ignored by mock parser"),
                              "SimulationBuilder did not use the injected parser.");

        auto& simulator = builder.build();
        test_support::require(
            simulator.getRegistry().has("vehicle.dynamics"),
            "Mission built through injected parser did not assemble the form component.");
        test_support::require(
            simulator.getRegistry().has("vehicle.interaction"),
            "Mission built through injected parser did not assemble the interaction component.");

        std::cout << "parser facade injection checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
