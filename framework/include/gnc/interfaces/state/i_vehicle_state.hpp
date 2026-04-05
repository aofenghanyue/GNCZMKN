/**
 * @file i_vehicle_state.hpp
 * @brief 飞行器状态接口
 * 
 * 定义飞行器状态的标准访问接口
 * 实现者包括 TruthState（真实状态）和 EstimatedState（估计状态）
 */
#pragma once

#include "gnc/common/math/math.hpp"

namespace gnc::interfaces {

using namespace gnc::math;

/**
 * @brief 大地坐标
 */
struct LLA {
    double lat;  ///< 纬度 (rad)
    double lon;  ///< 经度 (rad)
    double alt;  ///< 高度 (m)
    
    LLA() : lat(0), lon(0), alt(0) {}
    LLA(double lat_, double lon_, double alt_) 
        : lat(lat_), lon(lon_), alt(alt_) {}
};

/**
 * @brief 飞行器状态接口
 * 
 * 提供坐标变换所需的状态数据
 */
class IVehicleState {
public:
    virtual ~IVehicleState() = default;
    
    // ==================== 基本状态 ====================
    
    /// 位置 (ECEF, m)
    virtual Vector3 getPosition() const = 0;
    
    /// 速度 (ECEF, m/s)
    virtual Vector3 getVelocity() const = 0;
    
    /// 姿态四元数 (NED→Body)
    virtual Quaternion getAttitude() const = 0;
    
    /// 角速度 (Body系, rad/s)
    virtual Vector3 getAngularVelocity() const = 0;
    
    // ==================== 位置表示 ====================
    
    /// 大地坐标 (经纬高)
    virtual LLA getLLA() const = 0;
    
    // ==================== 气动参数 ====================
    
    /// 攻角 (rad)
    virtual double getAlpha() const = 0;
    
    /// 侧滑角 (rad)
    virtual double getBeta() const = 0;
    
    /// 总速度 (m/s)
    virtual double getAirspeed() const = 0;
    
    /// 马赫数
    virtual double getMach() const = 0;
    
    /// 动压 (Pa)
    virtual double getDynamicPressure() const = 0;
};

} // namespace gnc::interfaces
