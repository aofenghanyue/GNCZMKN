/**
 * @file math_types.hpp
 * @brief 数学类型定义
 */
#pragma once

#include <cmath>
#include <array>

namespace gnc {

/// 三维向量
struct Vector3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    Vector3d() = default;
    Vector3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    static Vector3d Zero() { return {0.0, 0.0, 0.0}; }
    
    Vector3d operator+(const Vector3d& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    
    Vector3d operator-(const Vector3d& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    
    Vector3d operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    
    Vector3d& operator+=(const Vector3d& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }
    
    double norm() const {
        return std::sqrt(x*x + y*y + z*z);
    }
};

/// 四元数 (姿态表示)
struct Quaterniond {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    Quaterniond() = default;
    Quaterniond(double w_, double x_, double y_, double z_) 
        : w(w_), x(x_), y(y_), z(z_) {}
    
    static Quaterniond Identity() { return {1.0, 0.0, 0.0, 0.0}; }
    
    void normalize() {
        double n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n > 1e-10) {
            w /= n; x /= n; y /= n; z /= n;
        }
    }
};

/// 3x3 矩阵
struct Matrix3d {
    std::array<std::array<double, 3>, 3> data = {{{1,0,0}, {0,1,0}, {0,0,1}}};
    
    static Matrix3d Identity() {
        Matrix3d m;
        return m;
    }
    
    static Matrix3d Zero() {
        Matrix3d m;
        m.data = {{{0,0,0}, {0,0,0}, {0,0,0}}};
        return m;
    }
    
    double& operator()(int row, int col) { return data[row][col]; }
    double operator()(int row, int col) const { return data[row][col]; }
};

} // namespace gnc
