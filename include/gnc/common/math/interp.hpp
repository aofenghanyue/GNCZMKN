/**
 * @file interp.hpp
 * @brief 插值与拟合库 - 支持多维查表、样条和最小二乘
 * 
 * 包含:
 * - 1D/2D/3D 查表 (线性插值)
 * - N 维通用查表
 * - 三次样条插值 / Akima 插值
 * - 多项式/线性最小二乘拟合
 * 
 * 特性:
 * - 支持非均匀网格
 * - 可配置外推策略 (Clamp/Linear/Error)
 * 
 * @note 查表类维护断点数据，为有状态类
 */
#pragma once

#include "eigen_types.hpp"
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace gnc::math {

// ==================== 插值算法与外推策略 ====================

/**
 * @brief 插值算法类型
 */
enum class InterpMethod {
    Linear,      ///< 线性插值
    Nearest,     ///< 最近邻
    CubicSpline, ///< 三次样条
    Akima        ///< Akima 插值 (平滑无过冲)
};

/**
 * @brief 外推策略
 */
enum class ExtrapPolicy {
    Clamp,       ///< 钳位到边界值
    Linear,      ///< 线性外推
    Error        ///< 抛出异常
};

namespace detail {

/**
 * @brief 二分查找断点索引
 * @return 最大的 i 使得 breaks[i] <= x，若 x < breaks[0] 返回 0
 */
inline size_t find_interval(const std::vector<double>& breaks, double x) {
    if (x <= breaks.front()) return 0;
    if (x >= breaks.back()) return breaks.size() - 2;
    
    auto it = std::upper_bound(breaks.begin(), breaks.end(), x);
    return static_cast<size_t>(it - breaks.begin() - 1);
}

/**
 * @brief 应用外推策略
 */
inline double apply_extrap(double x, double lo, double hi, 
                          ExtrapPolicy policy, bool& out_of_range) {
    out_of_range = false;
    if (x < lo) {
        out_of_range = true;
        if (policy == ExtrapPolicy::Clamp) return lo;
        if (policy == ExtrapPolicy::Error) {
            throw std::out_of_range("Interpolation: x below lower bound");
        }
    } else if (x > hi) {
        out_of_range = true;
        if (policy == ExtrapPolicy::Clamp) return hi;
        if (policy == ExtrapPolicy::Error) {
            throw std::out_of_range("Interpolation: x above upper bound");
        }
    }
    return x;
}

} // namespace detail

// ==================== 1D 查表 ====================

/**
 * @brief 一维查表类
 * 
 * 支持非均匀断点，线性/最近邻插值
 */
class LookupTable1D {
public:
    /**
     * @brief 构造查表
     * @param breaks 断点向量 (必须严格递增)
     * @param values 对应值向量 (长度等于 breaks)
     * @param method 插值方法 (Linear/Nearest)
     * @param extrap 外推策略
     */
    LookupTable1D(std::vector<double> breaks, std::vector<double> values,
                  InterpMethod method = InterpMethod::Linear,
                  ExtrapPolicy extrap = ExtrapPolicy::Clamp)
        : breaks_(std::move(breaks)), values_(std::move(values)),
          method_(method), extrap_(extrap) {
        if (breaks_.size() != values_.size() || breaks_.size() < 2) {
            throw std::invalid_argument("LookupTable1D: invalid data sizes");
        }
    }
    
    /**
     * @brief 查表求值
     */
    double lookup(double x) const {
        bool oor;
        double x_clamped = detail::apply_extrap(x, breaks_.front(), breaks_.back(), extrap_, oor);
        
        // 若外推策略是 Clamp 且超出范围，使用钳位后的 x
        if (extrap_ == ExtrapPolicy::Clamp) {
            x = x_clamped;
        }
        
        size_t i = detail::find_interval(breaks_, x);
        
        if (method_ == InterpMethod::Nearest) {
            // 最近邻
            double d0 = std::abs(x - breaks_[i]);
            double d1 = std::abs(x - breaks_[i + 1]);
            return (d0 <= d1) ? values_[i] : values_[i + 1];
        }
        
        // 线性插值
        double t = (x - breaks_[i]) / (breaks_[i + 1] - breaks_[i]);
        return values_[i] + t * (values_[i + 1] - values_[i]);
    }
    
    double operator()(double x) const { return lookup(x); }
    
    // 访问器
    const std::vector<double>& breaks() const { return breaks_; }
    const std::vector<double>& values() const { return values_; }
    
private:
    std::vector<double> breaks_;
    std::vector<double> values_;
    InterpMethod method_;
    ExtrapPolicy extrap_;
};

// ==================== 2D 查表 ====================

/**
 * @brief 二维查表类 (双线性插值)
 * 
 * 数据布局: data[i][j] 对应 (x_breaks[i], y_breaks[j])
 */
class LookupTable2D {
public:
    /**
     * @brief 构造 2D 查表
     * @param x_breaks X 断点
     * @param y_breaks Y 断点
     * @param data 二维数据 [nx][ny]
     */
    LookupTable2D(std::vector<double> x_breaks, 
                  std::vector<double> y_breaks,
                  std::vector<std::vector<double>> data,
                  InterpMethod method = InterpMethod::Linear,
                  ExtrapPolicy extrap = ExtrapPolicy::Clamp)
        : x_breaks_(std::move(x_breaks)), y_breaks_(std::move(y_breaks)),
          data_(std::move(data)), method_(method), extrap_(extrap) {
        if (x_breaks_.size() < 2 || y_breaks_.size() < 2) {
            throw std::invalid_argument("LookupTable2D: need at least 2 breakpoints");
        }
        if (data_.size() != x_breaks_.size()) {
            throw std::invalid_argument("LookupTable2D: data size mismatch");
        }
    }
    
    /**
     * @brief 双线性插值查表
     */
    double lookup(double x, double y) const {
        bool oor;
        x = detail::apply_extrap(x, x_breaks_.front(), x_breaks_.back(), extrap_, oor);
        y = detail::apply_extrap(y, y_breaks_.front(), y_breaks_.back(), extrap_, oor);
        
        size_t i = detail::find_interval(x_breaks_, x);
        size_t j = detail::find_interval(y_breaks_, y);
        
        double tx = (x - x_breaks_[i]) / (x_breaks_[i + 1] - x_breaks_[i]);
        double ty = (y - y_breaks_[j]) / (y_breaks_[j + 1] - y_breaks_[j]);
        
        // 双线性插值
        double v00 = data_[i][j];
        double v10 = data_[i + 1][j];
        double v01 = data_[i][j + 1];
        double v11 = data_[i + 1][j + 1];
        
        return (1 - tx) * (1 - ty) * v00 +
               tx * (1 - ty) * v10 +
               (1 - tx) * ty * v01 +
               tx * ty * v11;
    }
    
    double operator()(double x, double y) const { return lookup(x, y); }
    
private:
    std::vector<double> x_breaks_;
    std::vector<double> y_breaks_;
    std::vector<std::vector<double>> data_;
    InterpMethod method_;
    ExtrapPolicy extrap_;
};

// ==================== 3D 查表 ====================

/**
 * @brief 三维查表类 (三线性插值)
 */
class LookupTable3D {
public:
    LookupTable3D(std::vector<double> x_breaks,
                  std::vector<double> y_breaks,
                  std::vector<double> z_breaks,
                  std::vector<std::vector<std::vector<double>>> data,
                  ExtrapPolicy extrap = ExtrapPolicy::Clamp)
        : x_breaks_(std::move(x_breaks)), y_breaks_(std::move(y_breaks)),
          z_breaks_(std::move(z_breaks)), data_(std::move(data)), extrap_(extrap) {}
    
    double lookup(double x, double y, double z) const {
        bool oor;
        x = detail::apply_extrap(x, x_breaks_.front(), x_breaks_.back(), extrap_, oor);
        y = detail::apply_extrap(y, y_breaks_.front(), y_breaks_.back(), extrap_, oor);
        z = detail::apply_extrap(z, z_breaks_.front(), z_breaks_.back(), extrap_, oor);
        
        size_t i = detail::find_interval(x_breaks_, x);
        size_t j = detail::find_interval(y_breaks_, y);
        size_t k = detail::find_interval(z_breaks_, z);
        
        double tx = (x - x_breaks_[i]) / (x_breaks_[i + 1] - x_breaks_[i]);
        double ty = (y - y_breaks_[j]) / (y_breaks_[j + 1] - y_breaks_[j]);
        double tz = (z - z_breaks_[k]) / (z_breaks_[k + 1] - z_breaks_[k]);
        
        // 三线性插值 (8 个角点)
        double c000 = data_[i][j][k];
        double c100 = data_[i+1][j][k];
        double c010 = data_[i][j+1][k];
        double c110 = data_[i+1][j+1][k];
        double c001 = data_[i][j][k+1];
        double c101 = data_[i+1][j][k+1];
        double c011 = data_[i][j+1][k+1];
        double c111 = data_[i+1][j+1][k+1];
        
        double c00 = c000 * (1 - tx) + c100 * tx;
        double c01 = c001 * (1 - tx) + c101 * tx;
        double c10 = c010 * (1 - tx) + c110 * tx;
        double c11 = c011 * (1 - tx) + c111 * tx;
        
        double c0 = c00 * (1 - ty) + c10 * ty;
        double c1 = c01 * (1 - ty) + c11 * ty;
        
        return c0 * (1 - tz) + c1 * tz;
    }
    
    double operator()(double x, double y, double z) const { return lookup(x, y, z); }
    
private:
    std::vector<double> x_breaks_, y_breaks_, z_breaks_;
    std::vector<std::vector<std::vector<double>>> data_;
    ExtrapPolicy extrap_;
};

// ==================== 三次样条插值 ====================

/**
 * @brief 三次样条插值类
 * 
 * 自然边界条件 (端点二阶导为零)
 */
class CubicSpline {
public:
    /**
     * @brief 构造样条
     * @param x 断点 (必须严格递增)
     * @param y 对应值
     */
    CubicSpline(const std::vector<double>& x, const std::vector<double>& y,
                ExtrapPolicy extrap = ExtrapPolicy::Clamp)
        : x_(x), y_(y), extrap_(extrap) {
        if (x.size() != y.size() || x.size() < 2) {
            throw std::invalid_argument("CubicSpline: invalid data sizes");
        }
        compute_coefficients();
    }
    
    /**
     * @brief 样条求值
     */
    double eval(double x) const {
        bool oor;
        x = detail::apply_extrap(x, x_.front(), x_.back(), extrap_, oor);
        
        size_t i = detail::find_interval(x_, x);
        double dx = x - x_[i];
        
        return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }
    
    /**
     * @brief 一阶导数
     */
    double derivative(double x) const {
        bool oor;
        x = detail::apply_extrap(x, x_.front(), x_.back(), extrap_, oor);
        
        size_t i = detail::find_interval(x_, x);
        double dx = x - x_[i];
        
        return b_[i] + 2 * c_[i] * dx + 3 * d_[i] * dx * dx;
    }
    
    /**
     * @brief 二阶导数
     */
    double second_derivative(double x) const {
        bool oor;
        x = detail::apply_extrap(x, x_.front(), x_.back(), extrap_, oor);
        
        size_t i = detail::find_interval(x_, x);
        double dx = x - x_[i];
        
        return 2 * c_[i] + 6 * d_[i] * dx;
    }
    
    double operator()(double x) const { return eval(x); }
    
private:
    void compute_coefficients() {
        size_t n = x_.size();
        a_ = y_;
        b_.resize(n);
        c_.resize(n);
        d_.resize(n);
        
        std::vector<double> h(n - 1);
        for (size_t i = 0; i < n - 1; ++i) {
            h[i] = x_[i + 1] - x_[i];
        }
        
        // 三对角系统求解 c
        std::vector<double> alpha(n);
        for (size_t i = 1; i < n - 1; ++i) {
            alpha[i] = 3.0 / h[i] * (a_[i + 1] - a_[i]) - 
                       3.0 / h[i - 1] * (a_[i] - a_[i - 1]);
        }
        
        std::vector<double> l(n), mu(n), z(n);
        l[0] = 1.0;
        mu[0] = z[0] = 0.0;
        
        for (size_t i = 1; i < n - 1; ++i) {
            l[i] = 2.0 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        
        l[n - 1] = 1.0;
        z[n - 1] = c_[n - 1] = 0.0;
        
        for (int j = static_cast<int>(n) - 2; j >= 0; --j) {
            c_[j] = z[j] - mu[j] * c_[j + 1];
            b_[j] = (a_[j + 1] - a_[j]) / h[j] - h[j] * (c_[j + 1] + 2 * c_[j]) / 3.0;
            d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
        }
    }
    
    std::vector<double> x_, y_;
    std::vector<double> a_, b_, c_, d_;
    ExtrapPolicy extrap_;
};

// ==================== Akima 插值 ====================

/**
 * @brief Akima 插值类
 * 
 * 特点: 局部插值，避免样条的过冲问题，适合气动数据
 */
class AkimaSpline {
public:
    AkimaSpline(const std::vector<double>& x, const std::vector<double>& y,
                ExtrapPolicy extrap = ExtrapPolicy::Clamp)
        : x_(x), y_(y), extrap_(extrap) {
        if (x.size() != y.size() || x.size() < 2) {
            throw std::invalid_argument("AkimaSpline: invalid data sizes");
        }
        compute_coefficients();
    }
    
    double eval(double x) const {
        bool oor;
        x = detail::apply_extrap(x, x_.front(), x_.back(), extrap_, oor);
        
        size_t i = detail::find_interval(x_, x);
        double dx = x - x_[i];
        
        return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }
    
    double operator()(double x) const { return eval(x); }
    
private:
    void compute_coefficients() {
        size_t n = x_.size();
        a_ = y_;
        b_.resize(n - 1);
        c_.resize(n - 1);
        d_.resize(n - 1);
        
        // 计算斜率 m_i = (y_{i+1} - y_i) / (x_{i+1} - x_i)
        std::vector<double> m(n + 3);  // 扩展以处理边界
        for (size_t i = 0; i < n - 1; ++i) {
            m[i + 2] = (y_[i + 1] - y_[i]) / (x_[i + 1] - x_[i]);
        }
        // 边界外推
        m[1] = 2 * m[2] - m[3];
        m[0] = 2 * m[1] - m[2];
        m[n + 1] = 2 * m[n] - m[n - 1];
        m[n + 2] = 2 * m[n + 1] - m[n];
        
        // 计算 Akima 权重和导数
        std::vector<double> t(n);
        for (size_t i = 0; i < n; ++i) {
            double w1 = std::abs(m[i + 3] - m[i + 2]);
            double w2 = std::abs(m[i + 1] - m[i]);
            if (w1 + w2 < 1e-30) {
                t[i] = (m[i + 1] + m[i + 2]) / 2.0;
            } else {
                t[i] = (w1 * m[i + 1] + w2 * m[i + 2]) / (w1 + w2);
            }
        }
        
        // 计算系数
        for (size_t i = 0; i < n - 1; ++i) {
            double h = x_[i + 1] - x_[i];
            b_[i] = t[i];
            c_[i] = (3 * m[i + 2] - 2 * t[i] - t[i + 1]) / h;
            d_[i] = (t[i] + t[i + 1] - 2 * m[i + 2]) / (h * h);
        }
    }
    
    std::vector<double> x_, y_;
    std::vector<double> a_, b_, c_, d_;
    ExtrapPolicy extrap_;
};

// ==================== 最小二乘拟合 ====================

/**
 * @brief 多项式最小二乘拟合
 * @param x 自变量数据
 * @param y 因变量数据
 * @param degree 多项式阶数
 * @return 系数向量 [a0, a1, ..., an]，y = a0 + a1*x + a2*x^2 + ...
 */
inline VectorX lsq_fit_poly(const std::vector<double>& x, 
                            const std::vector<double>& y, 
                            int degree) {
    size_t n = x.size();
    MatrixX A(n, degree + 1);
    VectorX b(n);
    
    for (size_t i = 0; i < n; ++i) {
        double xi = 1.0;
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = xi;
            xi *= x[i];
        }
        b(i) = y[i];
    }
    
    return A.colPivHouseholderQr().solve(b);
}

/**
 * @brief 线性最小二乘拟合 y = a + b*x
 * @return (a, b) 截距和斜率
 */
inline std::pair<double, double> lsq_fit_linear(const std::vector<double>& x,
                                                 const std::vector<double>& y) {
    VectorX coeffs = lsq_fit_poly(x, y, 1);
    return {coeffs(0), coeffs(1)};
}

/**
 * @brief 多项式求值
 * @param coeffs 系数 [a0, a1, ..., an]
 * @param x 求值点
 */
inline double poly_eval(const VectorX& coeffs, double x) {
    double result = 0.0;
    double xi = 1.0;
    for (int i = 0; i < coeffs.size(); ++i) {
        result += coeffs(i) * xi;
        xi *= x;
    }
    return result;
}

} // namespace gnc::math
