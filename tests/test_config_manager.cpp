#include "gnc/core/config_manager.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cassert>

// Simple test framework
#define ASSERT_EQ(a, b) if ((a) != (b)) { std::cerr << "Assertion failed: " << #a << " == " << #b << " (" << (a) << " vs " << (b) << ")" << std::endl; return 1; }
#define ASSERT_TRUE(a) if (!(a)) { std::cerr << "Assertion failed: " << #a << std::endl; return 1; }
#define ASSERT_FALSE(a) if ((a)) { std::cerr << "Assertion failed: " << #a << std::endl; return 1; }
#define ASSERT_NEAR(a, b, eps) if (std::abs((a) - (b)) > (eps)) { std::cerr << "Assertion failed: " << #a << " near " << #b << " (" << (a) << " vs " << (b) << ")" << std::endl; return 1; }

using namespace gnc::core;

int test_valid_numbers() {
    std::cout << "Running test_valid_numbers..." << std::endl;
    ConfigManager manager;

    ASSERT_TRUE(manager.loadFromString("{\"int\": 123, \"neg\": -456, \"float\": 12.34, \"sci\": 1.5e3, \"sci_neg\": -2e-2}"));

    auto root = manager.root();
    ASSERT_EQ(root["int"].asInt(), 123);
    ASSERT_EQ(root["neg"].asInt(), -456);
    ASSERT_NEAR(root["float"].asDouble(), 12.34, 1e-9);
    ASSERT_NEAR(root["sci"].asDouble(), 1500.0, 1e-9);
    ASSERT_NEAR(root["sci_neg"].asDouble(), -0.02, 1e-9);

    std::cout << "test_valid_numbers passed." << std::endl;
    return 0;
}

int test_invalid_numbers() {
    std::cout << "Running test_invalid_numbers..." << std::endl;
    ConfigManager manager;

    // Malformed numbers should result in null nodes, not crashes

    // Case 1: "1e+"
    ASSERT_TRUE(manager.loadFromString("{\"val\": 1e+}"));
    ASSERT_TRUE(manager.root()["val"].isNull()); // Should be null because parsing failed

    // Case 2: "-"
    ASSERT_TRUE(manager.loadFromString("{\"val\": -}"));
    ASSERT_TRUE(manager.root()["val"].isNull());

    // Case 3: "1.2.3"
    ASSERT_TRUE(manager.loadFromString("{\"val\": 1.2.3}"));
    ASSERT_TRUE(manager.root()["val"].isNull());

    // Case 4: "-e"
    ASSERT_TRUE(manager.loadFromString("{\"val\": -e}"));
    ASSERT_TRUE(manager.root()["val"].isNull());

    std::cout << "test_invalid_numbers passed." << std::endl;
    return 0;
}

int test_infinite_loop_prevention() {
    std::cout << "Running test_infinite_loop_prevention..." << std::endl;
    ConfigManager manager;

    // This input caused infinite loop before fix because '.' was not handled
    // The parser should now encounter '.', skip it, and return null for value, then continue
    // Wait, if '.' is encountered where value is expected:
    // parseValue returns null. pos advanced by 1 ('.').
    // Next char is '}'. parseObject continues.

    // Input: {"val": .}
    // "val": [parseValue(".")] -> null
    // Object ends.

    ASSERT_TRUE(manager.loadFromString("{\"val\": .}"));
    ASSERT_TRUE(manager.root()["val"].isNull());

    std::cout << "test_infinite_loop_prevention passed." << std::endl;
    return 0;
}

int main() {
    if (test_valid_numbers() != 0) return 1;
    if (test_invalid_numbers() != 0) return 1;
    if (test_infinite_loop_prevention() != 0) return 1;

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
