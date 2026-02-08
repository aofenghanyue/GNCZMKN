/**
 * @file simple_dynamics.hpp
 * @brief 简单动力学模型
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/dynamics/i_dynamics.hpp"
#include "gnc/interfaces/disturbance/i_disturbance.hpp"
#include <vector>

namespace gnc::components {

/**
 * @brief 简单六自由度动力学
 * 
 * 占位实现，展示框架用法
 */
class SimpleDynamics : public core::ComponentBase,
                        public interfaces::IDynamics {
public:
    SimpleDynamics() : ComponentBase("SimpleDynamics") {
        // 动力学通常以最高频率运行
        setExecutionFrequency(0.0);  // 每步执行
    }
    
    // --- IDynamics 接口实现 ---
    
    const interfaces::VehicleState& getVehicleState() const override {
        return state_;
    }
    
    void setExternalForce(const Vector3d& force) override {
        external_force_ = force;
    }
    
    void setExternalTorque(const Vector3d& torque) override {
        external_torque_ = torque;
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
    
private:
    interfaces::VehicleState state_;
    Vector3d external_force_ = Vector3d::Zero();
    Vector3d external_torque_ = Vector3d::Zero();
    std::vector<interfaces::IDisturbance*> disturbances_;
};

// 自动注册到工厂
GNC_REGISTER_COMPONENT(SimpleDynamics, interfaces::IDynamics)

} // namespace gnc::components
