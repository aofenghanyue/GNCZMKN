# Volume 4: 核心引擎与状态管理 (Core Engine)

## 1. 状态布局管理器 (StateLayout)
传统框架中，飞行器状态通常被写死为一个结构体（例如 `struct State { double x, y, z, u, v, w; };`）。这导致当您想从 3DOF 切换到 6DOF，或者想加入额外的状态（如质量消耗、执行机构动态）时，必须修改核心代码。

本框架引入了 **`StateLayout` (状态布局管理器)**，彻底解决了这个问题。

### 工作机制
`StateLayout` 维护了一个字符串到整数索引的映射（`std::unordered_map<std::string, int>`），将动态名称映射到连续的 `Eigen::VectorXd` 数组索引上。

### 动力学模型中的使用示例
```cpp
Dynamics3DOF_SphericalEarth() {
    idx_lon_ = layout_.addVariable("longitude");
    idx_lat_ = layout_.addVariable("latitude");
    idx_h_   = layout_.addVariable("altitude");
    idx_v_   = layout_.addVariable("velocity");
    
    // 初始化连续内存状态向量
    state_ = Eigen::VectorXd::Zero(layout_.dimension());
}
```
外部组件（如制导或气动）可以不依赖特定的动力学类，直接通过接口安全获取状态：
```cpp
double mach = dynamics_->getStateValue("velocity") / 340.0;
```

## 2. 积分器架构 (IIntegrator)
在 `Simulator::step` 循环中，引擎对普通组件和 `IDynamicsModel` 组件区别对待。

如果组件继承了 `IDynamicsModel`，引擎不会直接调用 `update(dt)`，而是：
1. 取出当前状态 `x = dyn_model->getState()`。
2. 将 `dyn_model->computeDerivatives` 包装为 Lambda 闭包。
3. 交给当前的 `integrator_`（如 `RK4Integrator`）进行多步试探和微分方程求解。
4. 将推演后的新状态写回 `dyn_model->setState(x)`。

这种设计使得您可以随意切换积分算法（欧拉法、RK4、RK45），而无需修改哪怕一行物理模型代码。

## 3. 依赖验证 (DependencyValidator)
为了保证仿真在运行前是绝对安全的，组件可以实现 `IDependencyDeclarer` 接口，声明它“需要什么”。

```cpp
std::vector<core::DependencyDeclaration> getDependencies() const override {
    return {
        {std::type_index(typeid(interfaces::IAtmosphereModel)), "an atmosphere model", true},
        {std::type_index(typeid(interfaces::IGuidance3DOF)), "a 3DOF guidance", true}
    };
}
```
- 第三个参数 `true` 表示这是**硬依赖**。
- `SimulationBuilder` 构建完成后，`DependencyValidator::validate()` 会全量扫描。如果此时注册表中没有注册实现了 `IGuidance3DOF` 的组件，引擎会抛出详细的错误并拒绝启动，从根本上杜绝了主循环中的 `nullptr` 崩溃。