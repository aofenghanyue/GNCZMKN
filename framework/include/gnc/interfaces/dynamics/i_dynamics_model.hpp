#pragma once

#include "gnc/core/state_layout.hpp"
#include "gnc/common/math/eigen_types.hpp"

#include <string>

namespace gnc::interfaces {

/**
 * @brief 连续状态动力学模型接口
 *
 * 该接口用于声明“由仿真器统一积分推进”的连续状态组件。
 *
 * 调度约定：
 * - Simulator 不会依赖组件在 update() 中自行完成积分。
 * - Simulator 会在每个执行步读取状态、调用 computeDerivatives()、
 *   使用当前积分器推进状态，并回写 setState()。
 * - 如果具体组件同时继承 ComponentBase，则其 update() 可以保留给
 *   积分后的后处理逻辑（例如刷新缓存、发布派生量），但不应再次积分。
 */
class IDynamicsModel {
public:
    virtual ~IDynamicsModel() = default;

    virtual const gnc::core::StateLayout& getStateLayout() const = 0;
    virtual void computeDerivatives(double t,
                                    const Eigen::VectorXd& x,
                                    Eigen::VectorXd& dxdt) const = 0;
    virtual const Eigen::VectorXd& getState() const = 0;
    virtual void setState(const Eigen::VectorXd& x) = 0;
    virtual Eigen::VectorXd getInitialState() const = 0;

    // Generic string-based lookup is kept for logging, diagnostics,
    // stop conditions, and temporary tooling. Prefer typed provider
    // interfaces for cross-component algorithm dependencies.
    double getStateValue(const std::string& name) const {
        const int idx = getStateLayout().indexOf(name);
        const auto& state = getState();
        return (idx >= 0 && idx < state.size()) ? state[idx] : 0.0;
    }
};

} // namespace gnc::interfaces
