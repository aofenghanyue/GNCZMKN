/**
 * @file main.cpp
 * @brief GNC 仿真框架入口
 * 
 * 演示三行代码模式启动仿真
 */

#include "gnc/core/simulation_builder.hpp"
#include "gnc/common/logger.hpp"

// 包含组件头文件以触发自动注册
#include "gnc/components/environment/wgs84_earth.hpp"
#include "gnc/components/dynamics/simple_dynamics.hpp"
#include "gnc/components/sensors/ideal_imu.hpp"
#include "gnc/components/navigation/simple_navigation.hpp"

using namespace gnc::core;

int main() {
    gnc::log_info("=== GNC Simulation Framework ===");
    
    // ⭐ 三行代码模式
    SimulationBuilder builder;
    builder.loadConfig("config/simulation.json");  // 1. 加载配置
    auto& simulator = builder.build();              // 2. 构建仿真
    simulator.run();                                // 3. 运行仿真
    
    gnc::log_info("=== Simulation Completed ===");
    return 0;
}
