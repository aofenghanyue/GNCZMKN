#include "gnc/core/config_manager.hpp"
#include <iostream>
#include <cassert>

using namespace gnc::core;

void test_json_parsing() {
    std::string json = R"({
        "simulation": {
            "dt": 0.01,
            "duration": 10.0
        },
        "components": [
            { "name": "imu", "type": "IdealImu" },
            { "name": "gnc", "type": "SimpleGNC" }
        ],
        "enabled": true
    })";

    ConfigNode root = JsonParser::parse(json);

    assert(root.isObject());
    assert(root["simulation"].isObject());
    assert(root["simulation"]["dt"].asDouble() == 0.01);
    assert(root["simulation"]["duration"].asDouble() == 10.0);

    assert(root["components"].isArray());
    assert(root["components"].size() == 2);
    assert(root["components"][0]["name"].asString() == "imu");

    assert(root["enabled"].isBool());
    assert(root["enabled"].asBool() == true);

    std::cout << "[PASS] JSON Parsing" << std::endl;
}

void test_config_manager() {
    std::string json = R"({
        "simulation": { "dt": 0.001 },
        "components": [],
        "services": {}
    })";

    ConfigManager manager;
    bool success = manager.loadFromString(json);
    assert(success);

    assert(manager.simulation()["dt"].asDouble() == 0.001);

    std::cout << "[PASS] ConfigManager" << std::endl;
}

int main() {
    try {
        test_json_parsing();
        test_config_manager();
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
