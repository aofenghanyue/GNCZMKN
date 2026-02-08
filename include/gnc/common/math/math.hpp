/**
 * @file math.hpp
 * @brief 数学库统一导出
 * 
 * 包含：
 * - 基本类型 (Vector3, Matrix3, etc.)
 * - 四元数 (w,x,y,z 存储，右乘法则)
 * - 旋转算子
 * - 齐次变换
 * - 12种欧拉角序列
 * - 特殊函数 (Bessel, erf, sat, sign)
 * - 线性代数扩展 (SVD, pinv, Cholesky)
 * - 插值/拟合 (查表, 样条, 最小二乘)
 * - 数值积分/微分 (RK4/45, Jacobian)
 * - 方程求根 (Newton, 多项式)
 * - 概率统计 (高斯分布, 协方差)
 * - 优化算法 (黄金分割, Nelder-Mead)
 * - 非线性模型 (速率限制, 齿隙)
 */
#pragma once

// 核心类型与常量
#include "eigen_types.hpp"

// 姿态与变换
#include "quaternion.hpp"
#include "rotation.hpp"
#include "transform.hpp"
#include "euler_sequences.hpp"

// 特殊函数
#include "special_functions.hpp"

// 线性代数扩展
#include "linalg.hpp"

// 插值与拟合
#include "interp.hpp"

// 数值积分与微分
#include "calculus.hpp"

// 方程求根
#include "roots.hpp"

// 概率统计
#include "statistics.hpp"

// 优化算法
#include "optimization.hpp"

// 非线性模型
#include "nonlinear.hpp"

namespace gnc::math {

// 所有类型已在各自头文件中定义
// 此文件仅作为统一入口

} // namespace gnc::math

