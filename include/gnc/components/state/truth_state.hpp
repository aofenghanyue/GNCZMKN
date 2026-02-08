/**
 * @file truth_state.hpp
 * @brief 真实飞行器状态组件
 * 
 * 维护飞行器的真实物理状态（仿真真值）
 * 用于传感器误差建模、环境计算等
 */
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/state/i_vehicle_state.hpp"
#include "gnc/interfaces/dynamics/i_dynamics.hpp"
#include "gnc/libraries/coord/coord.hpp"
#include <cmath>

namespace gnc::components {

using namespace gnc::math;
using namespace gnc::coord;

/**
 * @brief 真实飞行器状态
 * 
 * 从 Dynamics 组件读取状态并计算派生量
 */
class TruthState : public core::ComponentBase,
                   public interfaces::IVehicleState {
public:
    TruthState() : ComponentBase("TruthState") {}
    
    // ==================== 生命周期 ====================
    
    void injectDependencies(core::ComponentRegistry& registry) override {
        dynamics_ = registry.getFirst<interfaces::IDynamics>();
    }
    
    void update(double dt) override {
        (void)dt;
        
        if (!dynamics_) return;
        
        // 从Dynamics获取基本状态
        position_ = dynamics_->getPosition();
        velocity_ = dynamics_->getVelocity();
        attitude_ = dynamics_->getAttitude();
        angularVelocity_ = dynamics_->getAngularVelocity();
        
        // 计算经纬高（简化版，假设球形地球）
        computeLLA();
        
        // 计算气动参数
        computeAeroParams();
    }
    
    // ==================== IVehicleState 实现 ====================
    
    Vector3 getPosition() const override { return position_; }
    Vector3 getVelocity() const override { return velocity_; }
    Quaternion getAttitude() const override { return attitude_; }
    Vector3 getAngularVelocity() const override { return angularVelocity_; }
    interfaces::LLA getLLA() const override { return lla_; }
    double getAlpha() const override { return alpha_; }
    double getBeta() const override { return beta_; }
    double getAirspeed() const override { return airspeed_; }
    double getMach() const override { return mach_; }
    double getDynamicPressure() const override { return dynamicPressure_; }

private:
    void computeLLA() {
        // 简化的ECEF转LLA（假设球形地球）
        constexpr double R_EARTH = 6378137.0;
        
        double x = position_.x();
        double y = position_.y();
        double z = position_.z();
        double p = std::sqrt(x*x + y*y);
        
        lla_.lon = std::atan2(y, x);
        lla_.lat = std::atan2(z, p);
        lla_.alt = std::sqrt(x*x + y*y + z*z) - R_EARTH;
    }
    
    void computeAeroParams() {
        // 将ECEF速度转换到Body系
        Matrix3 R_ecef_ned = ecef_to_ned_rotation(lla_.lat, lla_.lon);
        Matrix3 R_ned_body = attitude_.toRotationMatrix();
        Vector3 v_body = R_ned_body * R_ecef_ned * velocity_;
        
        // 计算气动参数
        airspeed_ = v_body.norm();
        
        if (airspeed_ > 1.0) {
            alpha_ = std::atan2(v_body.z(), v_body.x());
            beta_ = std::asin(v_body.y() / airspeed_);
        } else {
            alpha_ = 0.0;
            beta_ = 0.0;
        }
        
        // 简化的马赫数和动压计算
        constexpr double SPEED_OF_SOUND = 340.0;  // m/s
        constexpr double RHO = 1.225;             // kg/m³
        
        mach_ = airspeed_ / SPEED_OF_SOUND;
        dynamicPressure_ = 0.5 * RHO * airspeed_ * airspeed_;
    }
    
    // 依赖
    interfaces::IDynamics* dynamics_ = nullptr;
    
    // 基本状态
    Vector3 position_ = Vector3::Zero();
    Vector3 velocity_ = Vector3::Zero();
    Quaternion attitude_ = Quaternion::identity();
    Vector3 angularVelocity_ = Vector3::Zero();
    
    // 派生状态
    interfaces::LLA lla_;
    double alpha_ = 0.0;
    double beta_ = 0.0;
    double airspeed_ = 0.0;
    double mach_ = 0.0;
    double dynamicPressure_ = 0.0;
};

// 自动注册
GNC_REGISTER_COMPONENT(TruthState, interfaces::IVehicleState)

} // namespace gnc::components
