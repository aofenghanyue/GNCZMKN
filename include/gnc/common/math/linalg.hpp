/**
 * @file linalg.hpp
 * @brief 线性代数扩展 - 基于 Eigen 的高级线性代数操作
 * 
 * 包含:
 * - 伪逆 (Moore-Penrose)
 * - 奇异值分解 (SVD)
 * - Cholesky 分解
 * - 最小二乘求解
 * - 矩阵条件数
 * - 矩阵秩
 * 
 * @note 所有函数均为无状态纯函数，基于 Eigen3
 */
#pragma once

#include "eigen_types.hpp"
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace gnc::math {

// ==================== SVD 分解结果 ====================

/**
 * @brief 奇异值分解结果
 * 
 * A = U * S * V^T
 */
struct SvdResult {
    MatrixX U;      ///< 左奇异向量 (m x m 或 m x k，取决于选项)
    VectorX S;      ///< 奇异值向量 (k 个)
    MatrixX V;      ///< 右奇异向量 (n x n 或 n x k)
    int rank;       ///< 矩阵秩
};

// ==================== SVD 分解 ====================

/**
 * @brief 奇异值分解
 * @param A 输入矩阵 (m x n)
 * @param tol 秩判定容差 (默认使用机器精度)
 * @return SVD 分解结果
 */
inline SvdResult svd_decompose(const MatrixX& A, double tol = -1.0) {
    Eigen::JacobiSVD<MatrixX> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    SvdResult result;
    result.U = svd.matrixU();
    result.S = svd.singularValues();
    result.V = svd.matrixV();
    
    // 计算秩
    if (tol < 0.0) {
        tol = std::numeric_limits<double>::epsilon() * 
              std::max(A.rows(), A.cols()) * result.S(0);
    }
    result.rank = (result.S.array() > tol).count();
    
    return result;
}

/**
 * @brief 紧凑 SVD 分解 (只计算非零奇异值对应的向量)
 * @param A 输入矩阵
 * @return SVD 分解结果 (U: m x k, S: k, V: n x k)
 */
inline SvdResult svd_thin(const MatrixX& A) {
    Eigen::JacobiSVD<MatrixX> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    SvdResult result;
    result.U = svd.matrixU();
    result.S = svd.singularValues();
    result.V = svd.matrixV();
    result.rank = svd.rank();
    
    return result;
}

// ==================== 伪逆 ====================

/**
 * @brief Moore-Penrose 伪逆
 * 
 * 计算 A 的伪逆 A⁺，满足:
 * - A * A⁺ * A = A
 * - A⁺ * A * A⁺ = A⁺
 * - (A * A⁺)^T = A * A⁺
 * - (A⁺ * A)^T = A⁺ * A
 * 
 * @param A 输入矩阵 (m x n)
 * @param tol 奇异值截断容差 (小于此值视为零)
 * @return 伪逆矩阵 (n x m)
 */
inline MatrixX pinv(const MatrixX& A, double tol = -1.0) {
    Eigen::JacobiSVD<MatrixX> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    const VectorX& S = svd.singularValues();
    
    // 确定容差
    if (tol < 0.0) {
        tol = std::numeric_limits<double>::epsilon() * 
              std::max(A.rows(), A.cols()) * S(0);
    }
    
    // 计算 S⁺ (对角阵的伪逆)
    VectorX S_inv = VectorX::Zero(S.size());
    for (int i = 0; i < S.size(); ++i) {
        if (S(i) > tol) {
            S_inv(i) = 1.0 / S(i);
        }
    }
    
    // A⁺ = V * S⁺ * U^T
    return svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose();
}

// ==================== Cholesky 分解 ====================

/**
 * @brief Cholesky 分解
 * 
 * 对于正定矩阵 A，分解为 A = L * L^T，其中 L 是下三角矩阵
 * 
 * @param A 正定矩阵 (n x n)
 * @return 下三角矩阵 L
 * @throws std::runtime_error 如果矩阵不是正定的
 */
inline MatrixX cholesky(const MatrixX& A) {
    Eigen::LLT<MatrixX> llt(A);
    
    if (llt.info() == Eigen::NumericalIssue) {
        throw std::runtime_error("Cholesky decomposition failed: matrix is not positive definite");
    }
    
    return llt.matrixL();
}

/**
 * @brief 安全 Cholesky 分解 (返回成功标志)
 * @param A 输入矩阵
 * @param L 输出下三角矩阵
 * @return 分解是否成功
 */
inline bool cholesky_safe(const MatrixX& A, MatrixX& L) {
    Eigen::LLT<MatrixX> llt(A);
    
    if (llt.info() == Eigen::NumericalIssue) {
        return false;
    }
    
    L = llt.matrixL();
    return true;
}

/**
 * @brief LDLT 分解 (适用于半正定或不定矩阵)
 * 
 * 分解为 A = P^T * L * D * L^T * P
 * 
 * @param A 对称矩阵 (n x n)
 * @param L 输出: 单位下三角矩阵
 * @param D 输出: 对角矩阵 (以向量形式)
 * @return 分解是否成功
 */
inline bool ldlt_decompose(const MatrixX& A, MatrixX& L, VectorX& D) {
    Eigen::LDLT<MatrixX> ldlt(A);
    
    if (ldlt.info() != Eigen::Success) {
        return false;
    }
    
    L = ldlt.matrixL();
    D = ldlt.vectorD();
    return true;
}

// ==================== 最小二乘求解 ====================

/**
 * @brief 最小二乘求解 Ax = b
 * 
 * 求解 min ||Ax - b||² 的 x
 * 
 * @param A 系数矩阵 (m x n)
 * @param b 右端向量 (m)
 * @return 最小二乘解 (n)
 */
inline VectorX solve_lstsq(const MatrixX& A, const VectorX& b) {
    return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

/**
 * @brief 使用 QR 分解的最小二乘求解 (更快但精度略低)
 */
inline VectorX solve_lstsq_qr(const MatrixX& A, const VectorX& b) {
    return A.colPivHouseholderQr().solve(b);
}

/**
 * @brief 带约束的最小二乘 (法方程求解)
 * 
 * 当 A 是满列秩时，使用法方程 A^T*A*x = A^T*b
 */
inline VectorX solve_lstsq_normal(const MatrixX& A, const VectorX& b) {
    return (A.transpose() * A).ldlt().solve(A.transpose() * b);
}

// ==================== 矩阵分析 ====================

/**
 * @brief 计算矩阵条件数 (2-范数)
 * 
 * cond(A) = σ_max / σ_min
 * 
 * @param A 输入矩阵
 * @return 条件数 (∞ 表示矩阵奇异)
 */
inline double cond_number(const MatrixX& A) {
    Eigen::JacobiSVD<MatrixX> svd(A);
    const VectorX& S = svd.singularValues();
    
    if (S.size() == 0 || S(S.size() - 1) < std::numeric_limits<double>::epsilon()) {
        return std::numeric_limits<double>::infinity();
    }
    
    return S(0) / S(S.size() - 1);
}

/**
 * @brief 计算矩阵秩
 * @param A 输入矩阵
 * @param tol 容差 (默认使用机器精度)
 * @return 矩阵秩
 */
inline int rank(const MatrixX& A, double tol = -1.0) {
    Eigen::JacobiSVD<MatrixX> svd(A);
    const VectorX& S = svd.singularValues();
    
    if (tol < 0.0) {
        tol = std::numeric_limits<double>::epsilon() * 
              std::max(A.rows(), A.cols()) * S(0);
    }
    
    return (S.array() > tol).count();
}

/**
 * @brief 判断矩阵是否正定
 */
inline bool is_positive_definite(const MatrixX& A) {
    if (A.rows() != A.cols()) return false;
    
    Eigen::LLT<MatrixX> llt(A);
    return llt.info() != Eigen::NumericalIssue;
}

/**
 * @brief 对称化矩阵 (A + A^T) / 2
 */
inline MatrixX symmetrize(const MatrixX& A) {
    return (A + A.transpose()) * 0.5;
}

/**
 * @brief 确保矩阵正定 (通过添加小的对角项)
 * @param A 输入矩阵
 * @param eps 最小特征值保证
 * @return 正定化后的矩阵
 */
inline MatrixX ensure_positive_definite(const MatrixX& A, double eps = 1e-10) {
    MatrixX sym = symmetrize(A);
    Eigen::SelfAdjointEigenSolver<MatrixX> eig(sym);
    
    VectorX eigenvalues = eig.eigenvalues();
    double min_eig = eigenvalues.minCoeff();
    
    if (min_eig < eps) {
        return sym + (eps - min_eig) * MatrixX::Identity(A.rows(), A.cols());
    }
    return sym;
}

} // namespace gnc::math
