#include "test_support.hpp"

#include "gnc/core/config_manager.hpp"
#include "gnc/runset/config_json_writer.hpp"
#include "gnc/runset/matrix_case_source.hpp"
#include "gnc/runset/mission_overlay.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <unordered_map>

int main() {
    try {
        const char* mission = R"json(
{
  "simulation": { "dt": 1.0, "duration": 0.0 },
  "vehicles": [
    {
      "id": "vehicle",
      "perturbation": {
        "type": "perturbation.static",
        "name": "perturbation",
        "config": { "inputs": {} }
      },
      "form": { "components": [] },
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
    }
  ],
  "outputs": { "enabled": false }
}
)json";

        gnc::core::ConfigManager manager;
        test_support::require(manager.loadFromString(mission),
                              "Base mission JSON did not parse.");

        std::unordered_map<std::string, double> inputs{
            {"engine.temp_level", 2.0},
            {"aero.drag_bias", -0.03}
        };
        auto overlaid =
            gnc::runset::overlayVehiclePerturbationInputs(manager.root(),
                                                          "vehicle",
                                                          inputs);
        const auto text = gnc::runset::writeJson(overlaid);
        test_support::require(text.find("\"engine.temp_level\"") != std::string::npos,
                              "Overlay did not write engine.temp_level.");
        test_support::require(text.find("-0.03") != std::string::npos,
                              "Overlay did not write aero.drag_bias.");

        const std::string matrix =
            "case_id,engine.temp_level,aero.drag_bias\n"
            "hot,2,-0.03\n"
            "cold,0,0.02\n";
        const auto cases = gnc::runset::parseMatrixCases(matrix, {1});
        test_support::require(cases.size() == 1,
                              "Row selection should return one matrix case.");
        test_support::require(cases[0].case_id == "cold",
                              "Selected row case_id mismatch.");
        test_support::require(cases[0].inputs.at("engine.temp_level") == 0.0,
                              "Selected row engine.temp_level mismatch.");
        test_support::require(cases[0].inputs.at("aero.drag_bias") == 0.02,
                              "Selected row aero.drag_bias mismatch.");

        const std::string malformed_matrix =
            "case_id,engine.temp_level\n"
            "bad,1abc\n";
        bool malformed_failed = false;
        try {
            (void)gnc::runset::parseMatrixCases(malformed_matrix, {0});
        } catch (const std::exception&) {
            malformed_failed = true;
        }
        test_support::require(malformed_failed,
                              "Matrix parser should reject numeric cells with trailing text.");

        const std::string whitespace_matrix =
            "case_id,engine.temp_level\n"
            "ok,1.5 \t\n";
        const auto whitespace_cases =
            gnc::runset::parseMatrixCases(whitespace_matrix, {0});
        test_support::require(whitespace_cases[0].inputs.at("engine.temp_level") == 1.5,
                              "Matrix parser should allow trailing whitespace.");

        std::cout << "runset overlay checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
