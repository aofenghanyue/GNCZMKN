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
                                                    string("interaction.cartesian_3dof.direct_accel")),
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

        {
            gnc::core::ConfigManager manager;
            const char* commented_json = R"json({
              // User-authored configs can explain fields.
              "name": "aero // not a comment",
              "block": "value /* not a comment */",
              "numbers": [1.0, 2.0,],
              "object": {
                "enabled": true,
              },
              /*
                Block comments are allowed between fields.
              */
              "after_comment": 3.0,
            })json";
            test_support::require(manager.loadFromString(commented_json),
                                  "Builtin JSON parser should accept comments and trailing commas.");
            test_support::require(manager.root()["name"].asString() ==
                                      "aero // not a comment",
                                  "Line-comment marker inside string was altered.");
            test_support::require(manager.root()["block"].asString() ==
                                      "value /* not a comment */",
                                  "Block-comment marker inside string was altered.");
            test_support::require(manager.root()["numbers"].size() == 2,
                                  "Trailing comma in array produced an extra value.");
            test_support::requireNear(manager.root()["numbers"][1].asDouble(),
                                      2.0,
                                      1e-12,
                                      "Array value after comment sanitizing changed.");
            test_support::require(manager.root()["object"]["enabled"].asBool(),
                                  "Trailing comma in object prevented bool parse.");
            test_support::requireNear(manager.root()["after_comment"].asDouble(),
                                      3.0,
                                      1e-12,
                                      "Value after block comment did not parse.");
        }

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
