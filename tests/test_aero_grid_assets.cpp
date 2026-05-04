#include "test_support.hpp"

#include "gnc/vehicle/common/assets/aero_grid_asset.hpp"
#include "gnc/vehicle/common/components/aero_grid_asset_provider.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"
#include "gnc/vehicle/output/components/grid_aero.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <system_error>

namespace {

namespace fs = std::filesystem;

fs::path repoRoot() {
    return fs::path(__FILE__).lexically_normal().parent_path().parent_path();
}

std::string normalizePath(const fs::path& path) {
    return path.lexically_normal().generic_string();
}

bool contains(const std::string& text, const std::string& expected) {
    return text.find(expected) != std::string::npos;
}

std::string readText(const fs::path& path) {
    std::ifstream stream(path, std::ios::binary);
    test_support::require(stream.is_open(), "Failed to read " + normalizePath(path));
    return std::string((std::istreambuf_iterator<char>(stream)),
                       std::istreambuf_iterator<char>());
}

void writeFile(const fs::path& path, const std::string& text) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    test_support::require(!ec, "Failed to create " + normalizePath(path.parent_path()));
    std::ofstream stream(path, std::ios::binary);
    test_support::require(stream.is_open(), "Failed to write " + normalizePath(path));
    stream << text;
}

struct ScratchAssets {
    explicit ScratchAssets(fs::path root)
        : repo(std::move(root)),
          active_project(readText(repo / "user/active_project")),
          project_asset(repo / "user" / trim(active_project) / "data/aero/uri_test/project_asset.json"),
          user_data_asset(repo / "user/data/aero/uri_test/shared_asset.json") {
        cleanup();
        writeFile(project_asset, R"json({ "project_value": 4.0 })json");
        writeFile(user_data_asset, R"json({ "shared_value": 5.0 })json");
    }

    ~ScratchAssets() { cleanup(); }

    static std::string trim(std::string text) {
        while (!text.empty() &&
               (text.back() == '\n' || text.back() == '\r' || text.back() == ' ' ||
                text.back() == '\t')) {
            text.pop_back();
        }
        size_t first = 0;
        while (first < text.size() &&
               (text[first] == '\n' || text[first] == '\r' || text[first] == ' ' ||
                text[first] == '\t')) {
            ++first;
        }
        return text.substr(first);
    }

    void cleanup() const {
        std::error_code ec;
        fs::remove_all(project_asset.parent_path(), ec);
        ec.clear();
        fs::remove_all(user_data_asset.parent_path(), ec);
    }

    fs::path repo;
    std::string active_project;
    fs::path project_asset;
    fs::path user_data_asset;
};

gnc::core::ConfigNode loadJsonFile(const fs::path& path) {
    gnc::core::ConfigManager manager;
    test_support::require(manager.loadFromFile(path.string()),
                          "Failed to load " + normalizePath(path));
    return manager.root();
}

} // namespace

int main() {
    try {
        const fs::path root = repoRoot();
        ScratchAssets scratch(root);

        const auto project_resolved =
            gnc::vehicle::common::assets::resolveJsonAssetPath(
                "project://data/aero/uri_test/project_asset.json");
        test_support::require(project_resolved == fs::absolute(scratch.project_asset).lexically_normal(),
                              "project:// asset path did not resolve through active project.");

        const auto user_data_resolved =
            gnc::vehicle::common::assets::resolveJsonAssetPath(
                "user-data://aero/uri_test/shared_asset.json");
        test_support::require(user_data_resolved ==
                                  fs::absolute(scratch.user_data_asset).lexically_normal(),
                              "user-data:// asset path did not resolve through user/data.");

        const auto loaded_project_asset =
            gnc::vehicle::common::assets::loadConfiguredJsonAsset(
                test_support::object({
                    test_support::field(
                        "asset_file",
                        test_support::string(
                            "project://data/aero/uri_test/project_asset.json")),
                }),
                "test.asset");
        test_support::requireNear(loaded_project_asset["project_value"].asDouble(),
                                  4.0,
                                  1e-12,
                                  "Loaded project asset value changed.");

        const fs::path valid_grid = root / "user/data/aero/uri_test/valid_grid.json";
        writeFile(valid_grid,
                  R"json({
  "schema": "gnc.aero.grid.v1",
  "reference": {
    "area_m2": 2.0,
    "length_m": 3.0,
    "span_m": 4.0,
    "moment_ref_x_m": 1.5
  },
  "axes": [
    { "name": "mach", "nodes": [0.5, 1.0] },
    { "name": "alpha_rad", "nodes": [0.0, 0.1] }
  ],
  "layout": {
    "axis_order": ["mach", "alpha_rad"],
    "fastest_varying_axis": "alpha_rad"
  },
  "columns": ["c_axial", { "name": "c_normal" }, "c_pitch"],
  "data": [
    [0.10, 0.00, 0.00],
    [0.11, 0.20, -0.01],
    [0.20, 0.00, 0.00],
    [0.21, 0.40, -0.02]
  ]
})json");
        const auto asset =
            gnc::vehicle::common::assets::parseAeroGridAsset(loadJsonFile(valid_grid),
                                                             "valid_grid");
        test_support::require(asset.schema == "gnc.aero.grid.v1",
                              "Aero grid schema was not preserved.");
        test_support::requireNear(asset.reference.area_m2,
                                  2.0,
                                  1e-12,
                                  "Reference area did not parse.");
        test_support::require(asset.axes.size() == 2,
                              "Aero grid axes did not parse.");
        test_support::require(asset.axes[1].name == "alpha_rad",
                              "Second axis name did not parse.");
        test_support::require(asset.columns.size() == 3,
                              "String shorthand/object columns did not normalize.");
        test_support::require(asset.columns[1].name == "c_normal",
                              "Object-form column did not parse.");
        test_support::require(asset.columnIndex("c_pitch") == 2,
                              "Column lookup returned the wrong index.");
        test_support::require(asset.data.size() == 4,
                              "Aero grid data row count changed.");
        test_support::requireNear(asset.data[3][1],
                                  0.40,
                                  1e-12,
                                  "Aero grid matrix value changed.");

        const fs::path bad_grid = root / "user/data/aero/uri_test/bad_grid.json";
        writeFile(bad_grid,
                  R"json({
  "schema": "gnc.aero.grid.v1",
  "reference": { "area_m2": 2.0, "length_m": 3.0 },
  "axes": [
    { "name": "mach", "nodes": [0.5, 1.0] },
    { "name": "alpha_rad", "nodes": [0.0, 0.1] }
  ],
  "columns": ["c_axial"],
  "data": [[0.1], [0.2], [0.3]]
})json");
        bool bad_grid_failed = false;
        try {
            (void)gnc::vehicle::common::assets::parseAeroGridAsset(
                loadJsonFile(bad_grid),
                "bad_grid");
        } catch (const std::exception& ex) {
            bad_grid_failed = true;
            test_support::require(contains(ex.what(), "data row count"),
                                  "Bad grid failure should explain row count mismatch.");
        }
        test_support::require(bad_grid_failed,
                              "Aero grid parser should reject mismatched data rows.");

        gnc::vehicle::common::AeroGridAssetProvider provider;
        provider.configure(test_support::object({
                               test_support::field(
                                   "asset_file",
                                   test_support::string(
                                       "user-data://aero/uri_test/valid_grid.json")),
                           }),
                           "test.aero_asset.config");
        test_support::require(provider.getAeroGridAsset().columns[0].name ==
                                  "c_axial",
                              "Common aero asset provider did not expose loaded grid.");

        gnc::vehicle::output::GridAero direct_aero;
        direct_aero.configure(test_support::object({
                                  test_support::field(
                                      "asset_file",
                                      test_support::string(
                                          "user-data://aero/uri_test/valid_grid.json")),
                              }),
                              "test.aero.config");
        const auto direct_coefficients =
            direct_aero.computeCoefficients(0.05, 0.0, 0.75);
        const double expected_axial = 0.155;
        const double expected_normal = 0.15;
        const double expected_drag =
            expected_axial * std::cos(0.05) + expected_normal * std::sin(0.05);
        const double expected_lift =
            expected_normal * std::cos(0.05) - expected_axial * std::sin(0.05);
        test_support::requireNear(direct_coefficients.drag_coefficient,
                                  expected_drag,
                                  1e-12,
                                  "Direct aero.grid body-axis drag conversion failed.");
        test_support::requireNear(direct_coefficients.lift_coefficient,
                                  expected_lift,
                                  1e-12,
                                  "Direct aero.grid body-axis lift conversion failed.");
        test_support::requireNear(direct_coefficients.pitching_moment_coefficient,
                                  -0.0075,
                                  1e-12,
                                  "Direct aero.grid pitch coefficient interpolation failed.");

        gnc::core::ComponentRegistry registry;
        auto provider_component =
            std::make_unique<gnc::vehicle::common::AeroGridAssetProvider>();
        provider_component->configure(test_support::object({
                                          test_support::field(
                                              "asset_file",
                                              test_support::string(
                                                  "user-data://aero/uri_test/valid_grid.json")),
                                      }),
                                      "test.aero_asset.config");
        registry.add<gnc::vehicle::common::AeroGridAssetProvider,
                     gnc::vehicle::common::IAeroGridAssetProvider>(
            "vehicle.aero_asset",
            std::move(provider_component));
        gnc::vehicle::output::GridAero provider_aero;
        provider_aero.configure(test_support::object({
                                    test_support::field("asset",
                                                        test_support::string("aero_asset")),
                                }),
                                "test.provider_aero.config");
        gnc::core::ScopedRegistry scoped_registry("vehicle", registry, "vehicle.aero");
        provider_aero.injectDependencies(scoped_registry);
        const auto provider_coefficients =
            provider_aero.computeCoefficients(0.1, 0.0, 1.0);
        const double node_drag = 0.21 * std::cos(0.1) + 0.40 * std::sin(0.1);
        const double node_lift = 0.40 * std::cos(0.1) - 0.21 * std::sin(0.1);
        test_support::requireNear(provider_coefficients.drag_coefficient,
                                  node_drag,
                                  1e-12,
                                  "Provider aero.grid drag conversion failed.");
        test_support::requireNear(provider_coefficients.lift_coefficient,
                                  node_lift,
                                  1e-12,
                                  "Provider aero.grid lift conversion failed.");

        std::cout << "aero grid asset helper checks passed\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
