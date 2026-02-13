/**
 * @file simple_gravity_turn.cpp
 * @brief GNC 仿真框架使用示例 —— 简单重力转弯制导
 *
 * 本示例展示:
 *   1. 配置驱动: 通过 JSON 字符串配置初始条件和仿真参数
 *   2. 自定义算法: 实现一个简单的重力转弯 Guidance 组件
 *   3. 运行仿真 + 数据记录: 利用 CsvDataLogger 输出 CSV，用于后续绘图
 *
 * 编译 (在项目根目录下):
 *   mkdir -p build && cd build
 *   cmake .. -DCMAKE_BUILD_TYPE=Release
 *   make simple_gravity_turn
 *
 * 运行:
 *   ./bin/simple_gravity_turn
 *
 * 输出:
 *   gravity_turn_sim.csv  —— 仿真数据 (可用 Python/Matlab 绘图)
 */

// ============================================================
//  框架头文件
// ============================================================
#include "gnc/core/simulation_builder.hpp"
#include "gnc/core/data_logger.hpp"
#include "gnc/common/logger.hpp"

// 触发内置组件的工厂自动注册
#include "gnc/components/environment/wgs84_earth.hpp"
#include "gnc/components/dynamics/simple_dynamics.hpp"
#include "gnc/components/sensors/ideal_imu.hpp"
#include "gnc/components/navigation/simple_navigation.hpp"

#include <cmath>
#include <iostream>
#include <iomanip>

using namespace gnc;
using namespace gnc::core;
using namespace gnc::interfaces;

// ============================================================
//  第一步: 自定义 Guidance 组件 —— 简单重力转弯
// ============================================================

/**
 * @brief 简单重力转弯制导
 *
 * 策略:
 * - 垂直上升阶段: 施加恒定向上推力 (thrust_)
 * - 转弯阶段 (t > pitch_start_time_): 逐渐倾斜推力方向
 *   推力角: pitch_angle = pitch_rate * (t - pitch_start_time)
 *   Fx = thrust * sin(pitch_angle)
 *   Fz = thrust * cos(pitch_angle)
 */
class GravityTurnGuidance : public ComponentBase,
                            public IGuidance {
public:
    GravityTurnGuidance() : ComponentBase("GravityTurnGuidance") {
        setExecutionFrequency(50.0);  // 50 Hz
    }

    // --- 配置: 从 JSON 读取参数 ---
    void configure(const ConfigNode& config) override {
        thrust_          = config["thrust"].asDouble(50.0);
        pitch_start_time_ = config["pitch_start_time"].asDouble(2.0);
        pitch_rate_      = config["pitch_rate"].asDouble(0.05);
        max_pitch_angle_ = config["max_pitch_angle"].asDouble(1.2);
    }

    // --- 依赖注入: 获取导航模块 ---
    void injectDependencies(ScopedRegistry& registry) override {
        nav_ = registry.getByName<INavigation>("nav");
    }

    // --- 制导核心算法 ---
    void update(double dt) override {
        (void)dt;
        if (!nav_) return;

        const auto& state = nav_->getNavState();
        double t = state.timestamp;

        double pitch_angle = 0.0;
        if (t > pitch_start_time_) {
            pitch_angle = pitch_rate_ * (t - pitch_start_time_);
            if (pitch_angle > max_pitch_angle_) {
                pitch_angle = max_pitch_angle_;
            }
        }

        // 推力分解 (简化到 x-z 平面)
        // z 轴向上 (垂直方向), x 轴水平 (下射方向)
        cmd_.acceleration_cmd = Vector3d{
            thrust_ * std::sin(pitch_angle),   // 水平分量
            0.0,
            thrust_ * std::cos(pitch_angle)    // 垂直分量
        };
        cmd_.timestamp = t;
    }

    // --- IGuidance 接口 ---
    const GuidanceCommand& getGuidanceCommand() const override { return cmd_; }
    void setTarget(const Vector3d& target) override { target_ = target; }
    bool isActive() const override { return true; }

private:
    INavigation* nav_ = nullptr;
    GuidanceCommand cmd_;
    Vector3d target_;

    // 可配置参数
    double thrust_           = 50.0;   // 推力大小 (N), 假设质量1kg时等同加速度
    double pitch_start_time_ = 2.0;    // 开始转弯的时间 (s)
    double pitch_rate_       = 0.05;   // 转弯角速率 (rad/s)
    double max_pitch_angle_  = 1.2;    // 最大俯仰角 (rad)
};

// 注册到工厂: 让 SimulationBuilder 能通过字符串名称创建
GNC_REGISTER_COMPONENT(GravityTurnGuidance, IGuidance)

// ============================================================
//  第二步: 自定义 Controller 组件 —— 将制导指令转为外力
// ============================================================

/**
 * @brief 简单控制器
 *
 * 将 Guidance 输出的加速度指令直接转为外力施加到 Dynamics
 * (假设质量 = 1 kg, 所以 force = acceleration)
 */
class SimpleController : public ComponentBase,
                         public IController {
public:
    SimpleController() : ComponentBase("SimpleController") {
        setExecutionFrequency(50.0);
    }

    void injectDependencies(ScopedRegistry& registry) override {
        guidance_ = registry.getByName<IGuidance>("guidance");
        dynamics_ = registry.getByName<IDynamics>("dynamics");
    }

    void update(double dt) override {
        (void)dt;
        if (!guidance_ || !dynamics_) return;
        if (!guidance_->isActive()) return;

        const auto& g_cmd = guidance_->getGuidanceCommand();

        // 施加重力 (向下 -z 方向, 假设质量 1 kg)
        constexpr double g = 9.81;
        Vector3d gravity{0.0, 0.0, -g};

        // 总外力 = 推力指令 + 重力
        Vector3d total_force = g_cmd.acceleration_cmd + gravity;

        dynamics_->setExternalForce(total_force);

        ctrl_cmd_.force_cmd = total_force;
        ctrl_cmd_.timestamp = g_cmd.timestamp;
    }

    const ControlCommand& getControlCommand() const override { return ctrl_cmd_; }
    const ActuatorCommand& getActuatorCommand() const override { return act_cmd_; }
    bool isActive() const override { return true; }

private:
    IGuidance* guidance_ = nullptr;
    IDynamics* dynamics_ = nullptr;
    ControlCommand ctrl_cmd_;
    ActuatorCommand act_cmd_;
};

GNC_REGISTER_COMPONENT(SimpleController, IController)

// ============================================================
//  第三步: 配置 + 构建 + 运行
// ============================================================

int main() {
    LOG_INFO("=== GNC Example: Simple Gravity Turn ===");

    // -------------------------------------------------------
    // 1. JSON 配置字符串 (也可以写成文件用 loadConfig 加载)
    // -------------------------------------------------------
    const char* config_json = R"({
        "simulation": {
            "dt": 0.01,
            "duration": 30.0
        },
        "components": [
            {
                "type": "Wgs84Earth",
                "name": "earth",
                "config": {}
            },
            {
                "type": "SimpleDynamics",
                "name": "dynamics",
                "config": {}
            },
            {
                "type": "IdealImu",
                "name": "imu",
                "config": {
                    "frequency_hz": 100
                }
            },
            {
                "type": "SimpleNavigation",
                "name": "nav",
                "config": {}
            },
            {
                "type": "GravityTurnGuidance",
                "name": "guidance",
                "config": {
                    "thrust": 50.0,
                    "pitch_start_time": 2.0,
                    "pitch_rate": 0.05,
                    "max_pitch_angle": 1.2
                }
            },
            {
                "type": "SimpleController",
                "name": "controller",
                "config": {}
            }
        ]
    })";

    // -------------------------------------------------------
    // 2. 构建仿真
    // -------------------------------------------------------
    SimulationBuilder builder;
    builder.loadConfigString(config_json);
    auto& simulator = builder.build();

    // -------------------------------------------------------
    // 3. (可选) 设置数据记录器
    // -------------------------------------------------------
    auto& registry = simulator.getRegistry();
    auto* dynamics = registry.get<IDynamics>("dynamics");
    auto* guidance = registry.get<IGuidance>("guidance");

    CsvDataLogger logger;

    if (dynamics) {
        logger.registerTopic<VehicleState>(
            "dynamics",
            [dynamics]() -> const VehicleState& {
                return dynamics->getVehicleState();
            },
            {"pos.x", "pos.y", "pos.z", "vel.x", "vel.y", "vel.z"},
            [](const VehicleState& s) {
                std::ostringstream oss;
                oss << std::setprecision(10)
                    << s.position.x << "," << s.position.y << "," << s.position.z << ","
                    << s.velocity.x << "," << s.velocity.y << "," << s.velocity.z;
                return oss.str();
            }
        );
    }

    if (guidance) {
        logger.registerTopic<GuidanceCommand>(
            "guidance",
            [guidance]() -> const GuidanceCommand& {
                return guidance->getGuidanceCommand();
            },
            {"acc_cmd.x", "acc_cmd.y", "acc_cmd.z"},
            [](const GuidanceCommand& g) {
                std::ostringstream oss;
                oss << std::setprecision(10)
                    << g.acceleration_cmd.x << ","
                    << g.acceleration_cmd.y << ","
                    << g.acceleration_cmd.z;
                return oss.str();
            }
        );
    }

    logger.beginSession("gravity_turn_sim");

    // -------------------------------------------------------
    // 4. 运行仿真
    // -------------------------------------------------------
    simulator.initialize();

    double dt = 0.01;
    double duration = 30.0;
    int total_steps = static_cast<int>(duration / dt);

    LOG_INFO("Running {} steps (dt={}, duration={})", total_steps, dt, duration);

    for (int step = 0; step < total_steps; ++step) {
        simulator.step(step);

        double t = step * dt;

        // 每步记录数据
        logger.logStep(t);

        // 每 5 秒打印一次状态
        if (step % 500 == 0 && dynamics) {
            const auto& s = dynamics->getVehicleState();
            LOG_INFO("t={:.2f}s  pos=({:.1f}, {:.1f}, {:.1f})  vel=({:.2f}, {:.2f}, {:.2f})",
                     t, s.position.x, s.position.y, s.position.z,
                     s.velocity.x, s.velocity.y, s.velocity.z);
        }
    }

    simulator.finalize();
    logger.endSession();

    // -------------------------------------------------------
    // 5. 打印结果摘要
    // -------------------------------------------------------
    if (dynamics) {
        const auto& final_state = dynamics->getVehicleState();
        std::cout << "\n===== Simulation Complete =====\n"
                  << "Final position: ("
                  << final_state.position.x << ", "
                  << final_state.position.y << ", "
                  << final_state.position.z << ") m\n"
                  << "Final velocity: ("
                  << final_state.velocity.x << ", "
                  << final_state.velocity.y << ", "
                  << final_state.velocity.z << ") m/s\n"
                  << "Output file: gravity_turn_sim.csv\n"
                  << "===============================\n";
    }

    return 0;
}
