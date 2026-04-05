/**
 * @file rotations.hpp
 * @brief 坐标系旋转公式库
 * 
 * 设计理念：
 * - 只提供纯旋转公式
 * - 不包含具体天体参数
 * - 组件负责调用公式并提供参数
 * 
 * 数学约定（基于用户规格）：
 * - 坐标变换（被动）：v_new = M_{old→new} * v_old
 * - 左乘法则：M_total = M_last * ... * M_first
 */
#pragma once

#include "gnc/common/math/math.hpp"
#include <cmath>

namespace gnc::coord {

using namespace gnc::math;

// ==================== ECEF <-> NED/ENU 旋转矩阵 ====================

/**
 * @brief ECEF 转 NED 旋转矩阵
 * 
 * 纯公式：只需经纬度，不涉及具体天体
 * 
 * @param lat 纬度 (rad)
 * @param lon 经度 (rad)
 * @return 旋转矩阵 M_{ECEF→NED}
 */
inline Matrix3 ecef_to_ned_rotation(double lat, double lon) {
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);
    
    Matrix3 R;
    R << -sin_lat*cos_lon, -sin_lat*sin_lon,  cos_lat,
         -sin_lon,          cos_lon,           0,
         -cos_lat*cos_lon, -cos_lat*sin_lon, -sin_lat;
    return R;
}

/**
 * @brief NED 转 ECEF 旋转矩阵
 */
inline Matrix3 ned_to_ecef_rotation(double lat, double lon) {
    return ecef_to_ned_rotation(lat, lon).transpose();
}

/**
 * @brief ECEF 转 ENU 旋转矩阵
 */
inline Matrix3 ecef_to_enu_rotation(double lat, double lon) {
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);
    
    Matrix3 R;
    R << -sin_lon,          cos_lon,           0,
         -sin_lat*cos_lon, -sin_lat*sin_lon,  cos_lat,
          cos_lat*cos_lon,  cos_lat*sin_lon,  sin_lat;
    return R;
}

/**
 * @brief ENU 转 ECEF 旋转矩阵
 */
inline Matrix3 enu_to_ecef_rotation(double lat, double lon) {
    return ecef_to_enu_rotation(lat, lon).transpose();
}

// ==================== ECEF <-> NSE (North-Sky-East) 旋转矩阵 ====================

/**
 * @brief ECEF 转 NSE 旋转矩阵
 * 
 * 用户自定义局部坐标系（NED/ENU变种）
 * 轴定义：North, Sky(Up), East
 * 
 * 旋转序列：R_y(-90°) * R_x(φ) * R_z(λ-90°)
 * 
 * @param lat 纬度 φ (rad)
 * @param lon 经度 λ (rad)
 * @return 旋转矩阵 M_{ECEF→NSE}
 */
inline Matrix3 ecef_to_nse_rotation(double lat, double lon) {
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);
    
    // 公式来自用户推导脚本
    // Row 1 (North): [ -cos(lon)sin(lat),  -sin(lon)sin(lat),  cos(lat) ]
    // Row 2 (Sky):   [  cos(lon)cos(lat),   sin(lon)cos(lat),  sin(lat) ]
    // Row 3 (East):  [ -sin(lon),           cos(lon),          0        ]
    Matrix3 R;
    R << -cos_lon*sin_lat, -sin_lon*sin_lat,  cos_lat,
          cos_lon*cos_lat,  sin_lon*cos_lat,  sin_lat,
         -sin_lon,          cos_lon,          0;
    return R;
}

/**
 * @brief NSE 转 ECEF 旋转矩阵
 */
inline Matrix3 nse_to_ecef_rotation(double lat, double lon) {
    return ecef_to_nse_rotation(lat, lon).transpose();
}

// ==================== ECEF <-> Launch Frame 旋转矩阵 ====================

/**
 * @brief ECEF 转 Launch 坐标系旋转矩阵
 * 
 * 基于 NSE，绕 Sky(Y) 轴旋转方位角 A
 * 
 * 旋转序列：R_y(-90°-A) * R_x(φ) * R_z(λ-90°)
 * 
 * @param lat 纬度 φ (rad)
 * @param lon 经度 λ (rad)
 * @param azimuth 发射方位角 A (rad)
 * @return 旋转矩阵 M_{ECEF→Launch}
 */
inline Matrix3 ecef_to_launch_rotation(double lat, double lon, double azimuth) {
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);
    double sin_A = std::sin(azimuth);
    double cos_A = std::cos(azimuth);
    
    // 公式来自用户推导脚本
    Matrix3 R;
    R << -cos_A*cos_lon*sin_lat - sin_A*sin_lon,   sin_A*cos_lon - cos_A*sin_lon*sin_lat,   cos_A*cos_lat,
          cos_lon*cos_lat,                          sin_lon*cos_lat,                         sin_lat,
          sin_A*cos_lon*sin_lat - cos_A*sin_lon,   sin_A*sin_lon*sin_lat + cos_A*cos_lon,  -sin_A*cos_lat;
    return R;
}

/**
 * @brief Launch 转 ECEF 旋转矩阵
 */
inline Matrix3 launch_to_ecef_rotation(double lat, double lon, double azimuth) {
    return ecef_to_launch_rotation(lat, lon, azimuth).transpose();
}

// ==================== ECI/ECEF 旋转 ====================

/**
 * @brief ECI 转 ECEF 旋转矩阵
 * 
 * @param gast 格林尼治视恒星时 (rad)
 * @return 旋转矩阵
 */
inline Matrix3 eci_to_ecef_rotation(double gast) {
    double c = std::cos(gast);
    double s = std::sin(gast);
    
    Matrix3 R;
    R <<  c, s, 0,
         -s, c, 0,
          0, 0, 1;
    return R;
}

/**
 * @brief ECEF 转 ECI 旋转矩阵
 */
inline Matrix3 ecef_to_eci_rotation(double gast) {
    return eci_to_ecef_rotation(gast).transpose();
}

// ==================== Body/Wind/Stability 旋转 ====================

/**
 * @brief Body 转 NED 旋转矩阵（从姿态四元数）
 */
inline Matrix3 body_to_ned_dcm(const Quaternion& attitude) {
    // 按照用户约定：attitude 表示从某个参考系到 Body 的旋转
    // Body 到 NED 需要使用共轭
    return attitude.conjugate().toRotationMatrix();
}

/**
 * @brief NED 转 Body 旋转矩阵
 */
inline Matrix3 ned_to_body_dcm(const Quaternion& attitude) {
    return attitude.toRotationMatrix();
}

/**
 * @brief Body 转 Wind 旋转矩阵
 * 
 * @param alpha 攻角 (rad)
 * @param beta 侧滑角 (rad)
 */
inline Matrix3 body_to_wind_rotation(double alpha, double beta) {
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);
    double cb = std::cos(beta);
    double sb = std::sin(beta);
    
    Matrix3 R;
    R << ca*cb, sb, sa*cb,
        -ca*sb, cb, -sa*sb,
        -sa,    0,   ca;
    return R;
}

/**
 * @brief Wind 转 Body 旋转矩阵
 */
inline Matrix3 wind_to_body_rotation(double alpha, double beta) {
    return body_to_wind_rotation(alpha, beta).transpose();
}

/**
 * @brief Body 转 Stability 旋转矩阵
 * 
 * @param alpha 攻角 (rad)
 */
inline Matrix3 body_to_stability_rotation(double alpha) {
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);
    
    Matrix3 R;
    R << ca, 0, sa,
         0,  1,  0,
        -sa, 0, ca;
    return R;
}

// ==================== 辅助函数 ====================

/**
 * @brief 从速度计算攻角和侧滑角
 * 
 * @param v_body 载体系速度 (u, v, w)
 * @param alpha 输出攻角 (rad)
 * @param beta 输出侧滑角 (rad)
 */
inline void compute_alpha_beta(const Vector3& v_body, double& alpha, double& beta) {
    double u = v_body.x();
    double v = v_body.y();
    double w = v_body.z();
    double V = v_body.norm();
    
    if (V < 1e-6) {
        alpha = 0.0;
        beta = 0.0;
        return;
    }
    
    alpha = std::atan2(w, u);
    beta = std::asin(v / V);
}

/**
 * @brief 从欧拉角创建姿态四元数 (使用321序列)
 */
inline Quaternion euler_to_attitude(double t1, double t2, double t3) {
    return euler321ToQuat(t1, t2, t3);
}

/**
 * @brief 从姿态四元数提取欧拉角 (321序列)
 */
inline Vector3 attitude_to_euler(const Quaternion& q) {
    Matrix3 R = q.toRotationMatrix();
    
    // 321 序列反解
    double t2 = std::asin(-R(0, 2));
    double t1, t3;
    
    if (std::abs(std::cos(t2)) > 1e-10) {
        t1 = std::atan2(R(0, 1), R(0, 0));
        t3 = std::atan2(R(1, 2), R(2, 2));
    } else {
        // 万向节锁
        t1 = 0;
        t3 = std::atan2(-R(1, 0), R(1, 1));
    }
    
    return Vector3(t1, t2, t3);
}

} // namespace gnc::coord
