/**
 * @file rotation.hpp
 * @brief 旋转算子 - 统一的旋转表示
 * 
 * 设计理念：旋转是一个算子，可以用四元数、欧拉角或DCM表示
 * 内部使用四元数进行计算，提供统一的转换接口
 * 
 * 数学约定：
 * - 四元数右乘法则：q_total = q1 * q2
 * - 旋转公式：v' = q* ⊗ v ⊗ q
 */
#pragma once

#include "quaternion.hpp"
#include "euler_sequences.hpp"

namespace gnc::math {

/**
 * @brief 旋转算子类
 * 
 * 将旋转作为抽象算子，内部使用四元数存储和计算
 * 提供多种表示形式的转换接口
 * 
 * 使用示例：
 * @code
 * // 从欧拉角创建
 * Rotation R = Rotation::fromEuler(t1, t2, t3, EulerSeq::ZYX);
 * 
 * // 旋转向量
 * Vector3 v_new = R.apply(v_old);
 * 
 * // 组合旋转（右乘法则）
 * Rotation R_total = R1 * R2;  // 先 R1 再 R2
 * 
 * // 转换为其他表示
 * Matrix3 dcm = R.toDCM();
 * @endcode
 */
class Rotation {
public:
    // ==================== 构造函数 ====================
    
    /// 默认构造：单位旋转
    Rotation() : quat_(Quaternion::identity()) {}
    
    /// 从四元数构造
    explicit Rotation(const Quaternion& q) : quat_(q.normalized()) {}
    
    /// 从旋转矩阵构造
    explicit Rotation(const Matrix3& dcm) 
        : quat_(Quaternion::fromRotationMatrix(dcm)) {}
    
    // ==================== 工厂方法 ====================
    
    /// 单位旋转
    static Rotation identity() { return Rotation(); }
    
    /// 从欧拉角创建（支持12种序列）
    static Rotation fromEuler(double t1, double t2, double t3,
                              EulerSeq seq = EulerSeq::ZYX) {
        return Rotation(eulerToQuat(t1, t2, t3, seq));
    }
    
    /// 从轴角创建
    static Rotation fromAxisAngle(const Vector3& axis, double angle) {
        return Rotation(Quaternion::fromAxisAngle(axis, angle));
    }
    
    /// 从方向余弦矩阵创建
    static Rotation fromDCM(const Matrix3& dcm) {
        return Rotation(dcm);
    }
    
    /// 从四元数创建
    static Rotation fromQuaternion(const Quaternion& q) {
        return Rotation(q);
    }
    
    /// 绕 X 轴旋转
    static Rotation rotateX(double angle) {
        return Rotation(Quaternion::fromAxisAngle(Vector3(1, 0, 0), angle));
    }
    
    /// 绕 Y 轴旋转
    static Rotation rotateY(double angle) {
        return Rotation(Quaternion::fromAxisAngle(Vector3(0, 1, 0), angle));
    }
    
    /// 绕 Z 轴旋转
    static Rotation rotateZ(double angle) {
        return Rotation(Quaternion::fromAxisAngle(Vector3(0, 0, 1), angle));
    }
    
    // ==================== 旋转操作 ====================
    
    /// 应用旋转到向量（使用 q* ⊗ v ⊗ q）
    Vector3 apply(const Vector3& v) const {
        return quat_.rotate(v);
    }
    
    /// 逆旋转
    Vector3 applyInverse(const Vector3& v) const {
        return quat_.inverseRotate(v);
    }
    
    /// 运算符重载：旋转向量
    Vector3 operator*(const Vector3& v) const {
        return apply(v);
    }
    
    /// 旋转组合（右乘法则）
    Rotation operator*(const Rotation& other) const {
        return Rotation(quat_ * other.quat_);
    }
    
    /// 逆旋转
    Rotation inverse() const {
        return Rotation(quat_.conjugate());
    }
    
    // ==================== 转换到其他表示 ====================
    
    /// 转换为四元数
    const Quaternion& toQuaternion() const { return quat_; }
    
    /// 转换为方向余弦矩阵
    Matrix3 toDCM() const {
        return quat_.toRotationMatrix();
    }
    
    /// 转换为轴角
    void toAxisAngle(Vector3& axis, double& angle) const {
        Quaternion q = quat_.normalized();
        angle = 2.0 * std::acos(std::clamp(q.w(), -1.0, 1.0));
        
        double s = std::sqrt(1.0 - q.w()*q.w());
        if (s < constants::EPSILON) {
            axis = Vector3(1, 0, 0);
        } else {
            axis = Vector3(q.x()/s, q.y()/s, q.z()/s);
        }
    }
    
    // ==================== 运动学积分 ====================
    
    /// 使用角速度积分
    Rotation integrate(const Vector3& omega, double dt) const {
        return Rotation(quat_.integrate(omega, dt));
    }
    
private:
    Quaternion quat_;
};

} // namespace gnc::math
