/**
 * @file transform.hpp
 * @brief 齐次变换 (旋转 + 平移)
 */
#pragma once

#include "rotation.hpp"

namespace gnc::math {

/**
 * @brief 齐次变换类
 * 
 * 表示刚体变换：旋转 + 平移
 */
class Transform {
public:
    // ==================== 构造函数 ====================
    
    /// 默认构造：单位变换
    Transform() : rotation_(Rotation::identity()), translation_(Vector3::Zero()) {}
    
    /// 从旋转和平移构造
    Transform(const Rotation& R, const Vector3& t)
        : rotation_(R), translation_(t) {}
    
    /// 仅平移
    explicit Transform(const Vector3& t)
        : rotation_(Rotation::identity()), translation_(t) {}
    
    /// 仅旋转
    explicit Transform(const Rotation& R)
        : rotation_(R), translation_(Vector3::Zero()) {}
    
    // ==================== 工厂方法 ====================
    
    /// 单位变换
    static Transform identity() { return Transform(); }
    
    /// 从 4x4 矩阵创建
    static Transform fromMatrix4(const Matrix4& m) {
        Matrix3 R = m.block<3,3>(0,0);
        Vector3 t = m.block<3,1>(0,3);
        return Transform(Rotation::fromDCM(R), t);
    }
    
    // ==================== 访问器 ====================
    
    const Rotation& rotation() const { return rotation_; }
    Rotation& rotation() { return rotation_; }
    
    const Vector3& translation() const { return translation_; }
    Vector3& translation() { return translation_; }
    
    // ==================== 变换操作 ====================
    
    /// 变换点 (旋转 + 平移)
    Vector3 transformPoint(const Vector3& p) const {
        return rotation_.apply(p) + translation_;
    }
    
    /// 变换向量 (仅旋转)
    Vector3 transformVector(const Vector3& v) const {
        return rotation_.apply(v);
    }
    
    /// 运算符重载
    Vector3 operator*(const Vector3& p) const {
        return transformPoint(p);
    }
    
    /// 变换组合
    Transform operator*(const Transform& other) const {
        return Transform(
            rotation_ * other.rotation_,
            rotation_.apply(other.translation_) + translation_
        );
    }
    
    /// 逆变换
    Transform inverse() const {
        Rotation R_inv = rotation_.inverse();
        return Transform(R_inv, R_inv.apply(-translation_));
    }
    
    // ==================== 转换 ====================
    
    /// 转换为 4x4 齐次矩阵
    Matrix4 toMatrix4() const {
        Matrix4 m = Matrix4::Identity();
        m.block<3,3>(0,0) = rotation_.toDCM();
        m.block<3,1>(0,3) = translation_;
        return m;
    }

private:
    Rotation rotation_;
    Vector3 translation_;
};

} // namespace gnc::math
