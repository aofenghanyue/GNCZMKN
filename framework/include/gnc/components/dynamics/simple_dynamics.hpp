/**
 * @file simple_dynamics.hpp
 * @brief 简单动力学模型
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/config_manager.hpp"
#include "gnc/core/observable_helpers.hpp"
#include "gnc/interfaces/dynamics/i_dynamics.hpp"
#include "gnc/interfaces/disturbance/i_disturbance.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include <vector>

namespace gnc::components {

/**
 * @brief 简单六自由度动力学
 * 
 * 占位实现，展示框架用法
 */
class SimpleDynamics : public core::ComponentBase,
                        public interfaces::IDynamics,
                        public interfaces::IObservable {
public:
    SimpleDynamics() : ComponentBase("SimpleDynamics") {
        // 动力学通常以最高频率运行
        setExecutionFrequency(0.0);  // 每步执行
    }
    
    // --- IDynamics 接口实现 ---
    
    const interfaces::VehicleState& getVehicleState() const override {
        return state_;
    }
    
    void addForce(const Vector3d& force) override {
        external_force_ += force;
    }
    
    void addTorque(const Vector3d& torque) override {
        external_torque_ += torque;
    }
    
    void clearExternalForces() override {
        external_force_ = Vector3d::Zero();
        external_torque_ = Vector3d::Zero();
    }
    
    void reset() override {
        state_ = interfaces::VehicleState{};
        external_force_ = Vector3d::Zero();
        external_torque_ = Vector3d::Zero();
    }
    
    // --- 扰动管理 ---
    
    void addDisturbance(interfaces::IDisturbance* dist) {
        disturbances_.push_back(dist);
    }
    
    // --- ComponentBase 生命周期 ---

    void configure(const core::ConfigNode& config) override {
        if (config.isNull()) {
            return;
        }
        const auto& initial_position = config["initial_position"];
        if (initial_position.isArray() && initial_position.size() >= 3) {
            state_.position = Vector3d{
                initial_position[0].asDouble(0.0),
                initial_position[1].asDouble(0.0),
                initial_position[2].asDouble(0.0)
            };
        }
        const auto& initial_velocity = config["initial_velocity"];
        if (initial_velocity.isArray() && initial_velocity.size() >= 3) {
            state_.velocity = Vector3d{
                initial_velocity[0].asDouble(0.0),
                initial_velocity[1].asDouble(0.0),
                initial_velocity[2].asDouble(0.0)
            };
        }
    }
    
    void update(double dt) override {
        // 累加扰动
        Vector3d total_force_dist = Vector3d::Zero();
        Vector3d total_torque_dist = Vector3d::Zero();
        
        for (auto* dist : disturbances_) {
            if (dist && dist->isEnabled()) {
                auto d = dist->getDisturbance();
                total_force_dist += d.force;
                total_torque_dist += d.torque;
            }
        }
        
        // TODO: 实现真正的动力学积分
        // 简单欧拉积分示例
        Vector3d total_force = external_force_ + total_force_dist;
        
        // 假设质量为1kg（占位）
        Vector3d acceleration = total_force;
        state_.velocity += acceleration * dt;
        state_.position += state_.velocity * dt;
        state_.timestamp += dt;
    }

    std::vector<interfaces::ObservableField> getObservableFields() const override {
        core::ObservableFieldBuilder builder;
        builder.addVector3d("position", [this]() -> const Vector3d& { return state_.position; });
        builder.addVector3d("velocity", [this]() -> const Vector3d& { return state_.velocity; });
        builder.addQuaterniond("attitude", [this]() -> const Quaterniond& { return state_.attitude; });
        builder.addVector3d("angular_velocity", [this]() -> const Vector3d& { return state_.angular_velocity; });
        builder.addScalar("timestamp", [this]() { return state_.timestamp; });
        return builder.build();
    }
    
private:
    interfaces::VehicleState state_;
    Vector3d external_force_ = Vector3d::Zero();
    Vector3d external_torque_ = Vector3d::Zero();
    std::vector<interfaces::IDisturbance*> disturbances_;
};

// 自动注册到工厂
GNC_REGISTER_COMPONENT(SimpleDynamics, interfaces::IDynamics)

} // namespace gnc::components
