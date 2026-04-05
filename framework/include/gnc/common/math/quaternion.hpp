/**
 * @file quaternion.hpp
 * @brief 四元数类 - 使用 (w, x, y, z) 实部在前约定
 * 
 * 数学约定（基于用户规格）：
 * - 存储顺序: (w, x, y, z)，标量在前
 * - 旋转公式: v' = q* ⊗ v ⊗ q (共轭三明治)
 * - 组合法则: 右乘法则 q_total = q1 ⊗ q2 ⊗ ... ⊗ qn
 *   满足 M(q1*q2) = M(q2)*M(q1)
 */
#pragma once

#include "eigen_types.hpp"
#include <cmath>
#include <stdexcept>

namespace gnc::math {

/**
 * @brief 四元数类
 * 
 * 使用 (w, x, y, z) 存储顺序，其中 w 是实部（标量部分）
 * 
 * 数学定义：q = w + x*i + y*j + z*k
 */
class Quaternion {
public:
    // ==================== 构造函数 ====================
    
    /// 默认构造：单位四元数
    Quaternion() : w_(1.0), x_(0.0), y_(0.0), z_(0.0) {}
    
    /// 分量构造 (w, x, y, z)
    Quaternion(double w, double x, double y, double z)
        : w_(w), x_(x), y_(y), z_(z) {}
    
    /// 从 Eigen 四元数构造
    explicit Quaternion(const Eigen::Quaterniond& eq)
        : w_(eq.w()), x_(eq.x()), y_(eq.y()), z_(eq.z()) {}
    
    /// 单位四元数
    static Quaternion identity() { return Quaternion(1.0, 0.0, 0.0, 0.0); }
    
    // ==================== 访问器 ====================
    
    double w() const { return w_; }
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }
    
    double& w() { return w_; }
    double& x() { return x_; }
    double& y() { return y_; }
    double& z() { return z_; }
    
    /// 获取向量部分
    Vector3 vec() const { return Vector3(x_, y_, z_); }
    
    /// 转换为 Eigen 四元数
    Eigen::Quaterniond toEigen() const {
        return Eigen::Quaterniond(w_, x_, y_, z_);
    }
    
    /// 转换为数组 [w, x, y, z]
    Vector4 coeffs() const {
        return Vector4(w_, x_, y_, z_);
    }
    
    // ==================== 四元数运算 ====================
    
    /**
     * @brief 四元数乘法（右乘法则）
     * 
     * q_total = q1 * q2
     * 满足 M(q1*q2) = M(q2)*M(q1)
     * 
     * 即：先执行 q1 旋转，再执行 q2 旋转
     */
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w_*q.w_ - x_*q.x_ - y_*q.y_ - z_*q.z_,
            w_*q.x_ + x_*q.w_ + y_*q.z_ - z_*q.y_,
            w_*q.y_ - x_*q.z_ + y_*q.w_ + z_*q.x_,
            w_*q.z_ + x_*q.y_ - y_*q.x_ + z_*q.w_
        );
    }
    
    /// 模长
    double norm() const {
        return std::sqrt(w_*w_ + x_*x_ + y_*y_ + z_*z_);
    }
    
    /// 模长平方
    double squaredNorm() const {
        return w_*w_ + x_*x_ + y_*y_ + z_*z_;
    }
    
    /// 归一化
    Quaternion normalized() const {
        double n = norm();
        if (n < constants::EPSILON) {
            return identity();
        }
        return Quaternion(w_/n, x_/n, y_/n, z_/n);
    }
    
    /// 原地归一化
    void normalize() {
        double n = norm();
        if (n > constants::EPSILON) {
            w_ /= n; x_ /= n; y_ /= n; z_ /= n;
        }
    }
    
    /// 共轭 q* = (w, -x, -y, -z)
    Quaternion conjugate() const {
        return Quaternion(w_, -x_, -y_, -z_);
    }
    
    /// 逆 (对于单位四元数等于共轭)
    Quaternion inverse() const {
        double n2 = squaredNorm();
        if (n2 < constants::EPSILON) {
            throw std::runtime_error("Cannot invert zero quaternion");
        }
        return Quaternion(w_/n2, -x_/n2, -y_/n2, -z_/n2);
    }
    
    // ==================== 旋转操作 ====================
    
    /**
     * @brief 使用四元数旋转向量
     * 
     * 公式: v' = q* ⊗ v ⊗ q (共轭三明治)
     * 其中 v 表示为纯四元数 (0, vx, vy, vz)
     */
    Vector3 rotate(const Vector3& v) const {
        // v' = q* ⊗ v ⊗ q
        Quaternion qv(0, v.x(), v.y(), v.z());
        Quaternion result = conjugate() * qv * (*this);
        return Vector3(result.x_, result.y_, result.z_);
    }
    
    /// 逆旋转向量 (使用 q ⊗ v ⊗ q*)
    Vector3 inverseRotate(const Vector3& v) const {
        Quaternion qv(0, v.x(), v.y(), v.z());
        Quaternion result = (*this) * qv * conjugate();
        return Vector3(result.x_, result.y_, result.z_);
    }
    
    // ==================== 转换到其他表示 ====================
    
    /**
     * @brief 转换为旋转矩阵 (DCM)
     * 
     * 满足 M(q1*q2) = M(q2)*M(q1) 的约定
     */
    Matrix3 toRotationMatrix() const {
        double w2 = w_*w_, x2 = x_*x_, y2 = y_*y_, z2 = z_*z_;
        double wx = w_*x_, wy = w_*y_, wz = w_*z_;
        double xy = x_*y_, xz = x_*z_, yz = y_*z_;
        
        Matrix3 R;
        R << w2+x2-y2-z2,  2*(xy+wz),    2*(xz-wy),
             2*(xy-wz),    w2-x2+y2-z2,  2*(yz+wx),
             2*(xz+wy),    2*(yz-wx),    w2-x2-y2+z2;
        return R;
    }
    
    // ==================== 从其他表示构造 ====================
    
    /// 从旋转矩阵构造
    static Quaternion fromRotationMatrix(const Matrix3& R);
    
    /// 从轴角构造
    static Quaternion fromAxisAngle(const Vector3& axis, double angle);
    
    // ==================== 四元数积分 ====================
    
    /// 四元数运动学积分
    /// @param omega 角速度 (rad/s)
    /// @param dt 时间步长 (s)
    Quaternion integrate(const Vector3& omega, double dt) const;

private:
    double w_, x_, y_, z_;
};

// ==================== 实现 ====================

inline Quaternion Quaternion::fromRotationMatrix(const Matrix3& R) {
    // 使用 Shepperd 方法
    double tr = R.trace();
    Quaternion q;
    
    if (tr > 0) {
        double s = std::sqrt(tr + 1.0) * 2.0;
        q.w_ = 0.25 * s;
        q.x_ = (R(2,1) - R(1,2)) / s;
        q.y_ = (R(0,2) - R(2,0)) / s;
        q.z_ = (R(1,0) - R(0,1)) / s;
    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
        double s = std::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2.0;
        q.w_ = (R(2,1) - R(1,2)) / s;
        q.x_ = 0.25 * s;
        q.y_ = (R(0,1) + R(1,0)) / s;
        q.z_ = (R(0,2) + R(2,0)) / s;
    } else if (R(1,1) > R(2,2)) {
        double s = std::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2.0;
        q.w_ = (R(0,2) - R(2,0)) / s;
        q.x_ = (R(0,1) + R(1,0)) / s;
        q.y_ = 0.25 * s;
        q.z_ = (R(1,2) + R(2,1)) / s;
    } else {
        double s = std::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2.0;
        q.w_ = (R(1,0) - R(0,1)) / s;
        q.x_ = (R(0,2) + R(2,0)) / s;
        q.y_ = (R(1,2) + R(2,1)) / s;
        q.z_ = 0.25 * s;
    }
    
    return q.normalized();
}

inline Quaternion Quaternion::fromAxisAngle(const Vector3& axis, double angle) {
    double half_angle = angle * 0.5;
    double s = std::sin(half_angle);
    Vector3 n = axis.normalized();
    return Quaternion(std::cos(half_angle), n.x()*s, n.y()*s, n.z()*s);
}

inline Quaternion Quaternion::integrate(const Vector3& omega, double dt) const {
    double omega_norm = omega.norm();
    
    if (omega_norm < constants::EPSILON) {
        return *this;
    }
    
    // 使用指数映射进行精确积分
    double half_angle = omega_norm * dt * 0.5;
    double s = std::sin(half_angle) / omega_norm;
    
    Quaternion delta_q(std::cos(half_angle), 
                       omega.x()*s, omega.y()*s, omega.z()*s);
    
    // 右乘法则: q_new = q * delta_q
    return (*this * delta_q).normalized();
}

} // namespace gnc::math
