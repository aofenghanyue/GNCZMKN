/**
 * @file state_space.hpp
 * @brief 状态空间模型表示和仿真
 * 
 * 支持连续和离散系统:
 * - 连续: x' = Ax + Bu, y = Cx + Du
 * - 离散: x[k+1] = Ad*x[k] + Bd*u[k], y = Cd*x[k] + Dd*u[k]
 */
#pragma once

#include "gnc/common/math/eigen_types.hpp"
#include <stdexcept>

namespace gnc::libraries::control {

using namespace gnc::math;

/**
 * @brief 系统类型
 */
enum class SystemType {
    Continuous,  ///< 连续时间系统
    Discrete     ///< 离散时间系统
};

/**
 * @brief 状态空间模型
 */
class StateSpaceModel {
public:
    /**
     * @brief 默认构造 (SISO, 1阶)
     */
    StateSpaceModel() {
        A_ = MatrixX::Zero(1, 1);
        B_ = MatrixX::Zero(1, 1);
        C_ = MatrixX::Identity(1, 1);
        D_ = MatrixX::Zero(1, 1);
        x_ = VectorX::Zero(1);
    }
    
    /**
     * @brief 从矩阵构造
     * @param A 系统矩阵 (n x n)
     * @param B 输入矩阵 (n x m)
     * @param C 输出矩阵 (p x n)
     * @param D 直通矩阵 (p x m)
     * @param type 系统类型
     */
    StateSpaceModel(const MatrixX& A, const MatrixX& B,
                    const MatrixX& C, const MatrixX& D,
                    SystemType type = SystemType::Continuous)
        : A_(A), B_(B), C_(C), D_(D), type_(type) {
        validate();
        x_ = VectorX::Zero(A_.rows());
    }
    
    /**
     * @brief 简化构造 (D = 0)
     */
    StateSpaceModel(const MatrixX& A, const MatrixX& B, const MatrixX& C)
        : A_(A), B_(B), C_(C), type_(SystemType::Continuous) {
        D_ = MatrixX::Zero(C_.rows(), B_.cols());
        validate();
        x_ = VectorX::Zero(A_.rows());
    }
    
    // ==================== 访问器 ====================
    
    const MatrixX& A() const { return A_; }
    const MatrixX& B() const { return B_; }
    const MatrixX& C() const { return C_; }
    const MatrixX& D() const { return D_; }
    
    MatrixX& A() { return A_; }
    MatrixX& B() { return B_; }
    MatrixX& C() { return C_; }
    MatrixX& D() { return D_; }
    
    int order() const { return static_cast<int>(A_.rows()); }      ///< 系统阶数
    int numInputs() const { return static_cast<int>(B_.cols()); }  ///< 输入维度
    int numOutputs() const { return static_cast<int>(C_.rows()); } ///< 输出维度
    
    SystemType type() const { return type_; }
    bool isContinuous() const { return type_ == SystemType::Continuous; }
    bool isDiscrete() const { return type_ == SystemType::Discrete; }
    
    // ==================== 状态操作 ====================
    
    const VectorX& state() const { return x_; }
    VectorX& state() { return x_; }
    
    void setState(const VectorX& x) {
        if (x.size() != x_.size()) {
            throw std::invalid_argument("State dimension mismatch");
        }
        x_ = x;
    }
    
    void resetState() { x_.setZero(); }
    
    // ==================== 仿真 ====================
    
    /**
     * @brief 计算输出 y = Cx + Du
     */
    VectorX output(const VectorX& u) const {
        return C_ * x_ + D_ * u;
    }
    
    /**
     * @brief 计算状态导数 (连续系统)
     */
    VectorX derivative(const VectorX& u) const {
        return A_ * x_ + B_ * u;
    }
    
    /**
     * @brief 离散系统一步更新
     * @param u 输入向量
     * @return 输出向量
     */
    VectorX updateDiscrete(const VectorX& u) {
        VectorX y = output(u);
        x_ = A_ * x_ + B_ * u;
        return y;
    }
    
    /**
     * @brief 连续系统一步更新 (Euler 积分)
     */
    VectorX updateContinuous(const VectorX& u, double dt) {
        VectorX y = output(u);
        x_ += derivative(u) * dt;
        return y;
    }
    
    /**
     * @brief 连续系统一步更新 (RK4 积分)
     */
    VectorX updateContinuousRK4(const VectorX& u, double dt) {
        VectorX y = output(u);
        
        VectorX k1 = A_ * x_ + B_ * u;
        VectorX k2 = A_ * (x_ + 0.5 * dt * k1) + B_ * u;
        VectorX k3 = A_ * (x_ + 0.5 * dt * k2) + B_ * u;
        VectorX k4 = A_ * (x_ + dt * k3) + B_ * u;
        
        x_ += dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
        return y;
    }
    
    /**
     * @brief 通用更新方法
     */
    VectorX update(const VectorX& u, double dt = 0.0) {
        if (type_ == SystemType::Discrete) {
            return updateDiscrete(u);
        } else {
            return updateContinuousRK4(u, dt);
        }
    }
    
    // ==================== 系统分析 ====================
    
    /**
     * @brief 计算极点 (特征值)
     */
    Eigen::VectorXcd poles() const {
        Eigen::EigenSolver<MatrixX> es(A_);
        return es.eigenvalues();
    }
    
    /**
     * @brief 检查稳定性
     */
    bool isStable() const {
        auto p = poles();
        for (int i = 0; i < p.size(); ++i) {
            if (type_ == SystemType::Continuous) {
                if (p(i).real() >= 0) return false;
            } else {
                if (std::abs(p(i)) >= 1.0) return false;
            }
        }
        return true;
    }
    
private:
    void validate() {
        int n = A_.rows();
        if (A_.cols() != n) {
            throw std::invalid_argument("A must be square");
        }
        if (B_.rows() != n) {
            throw std::invalid_argument("B row count must match A");
        }
        if (C_.cols() != n) {
            throw std::invalid_argument("C column count must match A");
        }
        if (D_.rows() != C_.rows() || D_.cols() != B_.cols()) {
            throw std::invalid_argument("D dimensions must be compatible");
        }
    }
    
    MatrixX A_, B_, C_, D_;
    VectorX x_;
    SystemType type_ = SystemType::Continuous;
};

} // namespace gnc::libraries::control
