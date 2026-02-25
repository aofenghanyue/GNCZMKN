/**
 * @file eigen_types.hpp
 * @brief Eigen 封装和基本类型定义
 * 
 * 提供 GNC 框架使用的标准数学类型，基于 Eigen 库
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace gnc::math {

// ==================== 基本类型别名 ====================

/// 二维向量
using Vector2 = Eigen::Vector2d;

/// 三维向量
using Vector3 = Eigen::Vector3d;

/// 四维向量
using Vector4 = Eigen::Vector4d;

/// 六维向量 (位置+速度 或 力+力矩)
using Vector6 = Eigen::Matrix<double, 6, 1>;

/// 2x2 矩阵
using Matrix2 = Eigen::Matrix2d;

/// 3x3 矩阵
using Matrix3 = Eigen::Matrix3d;

/// 4x4 矩阵
using Matrix4 = Eigen::Matrix4d;

/// 6x6 矩阵
using Matrix6 = Eigen::Matrix<double, 6, 6>;

/// 动态大小矩阵
using MatrixX = Eigen::MatrixXd;

/// 动态大小向量
using VectorX = Eigen::VectorXd;

// ==================== 常用常量 ====================

namespace constants {

/// 圆周率
constexpr double PI = 3.14159265358979323846;

/// 2π
constexpr double TWO_PI = 2.0 * PI;

/// π/2
constexpr double HALF_PI = PI / 2.0;

/// 角度转弧度系数
constexpr double DEG_TO_RAD = PI / 180.0;

/// 弧度转角度系数
constexpr double RAD_TO_DEG = 180.0 / PI;

/// 数值容差
constexpr double EPSILON = 1e-12;

} // namespace constants

// ==================== 角度/弧度转换 ====================

/// 角度转弧度
inline double deg2rad(double deg) {
    return deg * constants::DEG_TO_RAD;
}

/// 弧度转角度
inline double rad2deg(double rad) {
    return rad * constants::RAD_TO_DEG;
}

/// 角度归一化到 [-π, π]
inline double normalizeAngle(double angle) {
    return std::remainder(angle, constants::TWO_PI);
}

/// 角度归一化到 [0, 2π]
inline double normalizeAngle2Pi(double angle) {
    angle = std::fmod(angle, constants::TWO_PI);
    if (angle < 0) angle += constants::TWO_PI;
    return angle;
}

// ==================== 矩阵工具 ====================

/// 创建反对称矩阵 (叉乘矩阵)
inline Matrix3 skew(const Vector3& v) {
    Matrix3 m;
    m <<     0, -v.z(),  v.y(),
         v.z(),      0, -v.x(),
        -v.y(),  v.x(),      0;
    return m;
}

/// 从反对称矩阵提取向量
inline Vector3 vex(const Matrix3& m) {
    return Vector3(m(2,1), m(0,2), m(1,0));
}

/// 单位矩阵
inline Matrix3 eye3() { return Matrix3::Identity(); }
inline Matrix4 eye4() { return Matrix4::Identity(); }

/// 零向量
inline Vector3 zeros3() { return Vector3::Zero(); }

} // namespace gnc::math
