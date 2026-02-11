#include "gnc/core/data_logger.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <cassert>

// Simple assertions for standalone test
#define ASSERT_EQ(a, b) if((a) != (b)) { std::cerr << "Assertion failed: " << #a << " (" << (a) << ") != " << #b << " (" << (b) << ")" << std::endl; std::exit(1); }
#define ASSERT_TRUE(a) if(!(a)) { std::cerr << "Assertion failed: " << #a << std::endl; std::exit(1); }

struct MockData {
    double x, y, z;
};

void test_basic_logging() {
    std::string filename = "test_basic_log";
    gnc::core::CsvDataLogger logger;
    MockData data{1.0, 2.0, 3.0};

    logger.registerTopic<MockData>("mock",
        [&data]() -> const MockData& { return data; },
        {"x", "y", "z"},
        [](const MockData& d) {
            std::ostringstream oss;
            oss << d.x << "," << d.y << "," << d.z;
            return oss.str();
        }
    );

    logger.beginSession(filename);
    ASSERT_TRUE(logger.isRecording());

    logger.logStep(0.1);
    data = {4.0, 5.0, 6.0};
    logger.logStep(0.2);

    logger.endSession();
    ASSERT_TRUE(!logger.isRecording());

    std::ifstream file(filename + ".csv");
    ASSERT_TRUE(file.is_open());

    std::string line;
    std::getline(file, line);
    ASSERT_EQ(line, "time,mock.x,mock.y,mock.z");

    std::getline(file, line);
    // Precision issue might occur, but with integers it should be exact or close enough for string comparison
    ASSERT_TRUE(line.find("1,2,3") != std::string::npos);

    std::getline(file, line);
    ASSERT_TRUE(line.find("4,5,6") != std::string::npos);

    file.close();
    std::remove((filename + ".csv").c_str());
    std::cout << "test_basic_logging passed" << std::endl;
}

void test_flush_every_step() {
    std::string filename = "test_flush_log";
    gnc::core::CsvDataLogger logger;
    MockData data{10.0, 20.0, 30.0};

    logger.registerTopic<MockData>("mock",
        [&data]() -> const MockData& { return data; },
        {"val"},
        [](const MockData& d) { return std::to_string(d.x); } // Simplified formatter
    );

    logger.setFlushEveryStep(true);
    logger.beginSession(filename);

    logger.logStep(0.5);

    // Read immediately without closing logger
    std::ifstream file(filename + ".csv");
    ASSERT_TRUE(file.is_open());

    std::string line;
    std::getline(file, line); // Header
    std::getline(file, line); // Data

    // If flush works, data should be present
    ASSERT_TRUE(!line.empty());
    ASSERT_TRUE(line.find("10.000000") != std::string::npos);

    file.close();
    logger.endSession();
    std::remove((filename + ".csv").c_str());
    std::cout << "test_flush_every_step passed" << std::endl;
}

int main() {
    test_basic_logging();
    test_flush_every_step();
    return 0;
}
