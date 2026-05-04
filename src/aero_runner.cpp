#include "gnc/core/config_manager.hpp"
#include "gnc/vehicle/common/assets/aero_grid_asset.hpp"
#include "gnc/vehicle/common/assets/json_asset_loader.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

void printUsage() {
    std::cerr << "Usage:\n"
              << "  gnc_aero inspect --asset <aero_grid.json>\n";
}

int inspectAsset(const std::string& configured_asset_path) {
    const auto resolved_path =
        gnc::vehicle::common::assets::resolveJsonAssetPath(configured_asset_path);

    gnc::core::ConfigManager manager;
    if (!manager.loadFromFile(resolved_path.string())) {
        throw std::runtime_error("Failed to load asset '" +
                                 resolved_path.generic_string() + "'.");
    }

    const auto asset = gnc::vehicle::common::assets::parseAeroGridAsset(
        manager.root(),
        "gnc_aero inspect asset '" + resolved_path.generic_string() + "'");

    std::cout << "schema: " << asset.schema << "\n";
    std::cout << "asset: " << resolved_path.generic_string() << "\n";
    std::cout << "reference_area_m2: " << asset.reference.area_m2 << "\n";
    std::cout << "reference_length_m: " << asset.reference.length_m << "\n";
    std::cout << "axes: " << asset.axes.size() << "\n";
    for (const auto& axis : asset.axes) {
        std::cout << "  " << axis.name << ": " << axis.nodes.size()
                  << " nodes [" << axis.nodes.front() << ", "
                  << axis.nodes.back() << "]\n";
    }
    std::cout << "columns: " << asset.columns.size() << "\n";
    for (const auto& column : asset.columns) {
        std::cout << "  " << column.name << "\n";
    }
    std::cout << "rows: " << asset.data.size() << "\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 4 && std::string(argv[1]) == "inspect" &&
            std::string(argv[2]) == "--asset") {
            return inspectAsset(argv[3]);
        }

        printUsage();
        return 2;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
