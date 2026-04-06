# 整体架构与引擎微设计剖析

本篇不仅带您俯瞰框架的宏观五层架构，更将深入探讨支撑该架构运转的底层 C++ 引擎微设计（Micro-designs）。本框架的初衷是在保持极高运行性能的同时，彻底解耦算法开发与底层设施。

---

## 1. 五层架构模型

框架从底至上分为五大层级：

1.  **数学库层 (Math Library)**：基于 `Eigen3` 封装，提供四元数、旋转算子、RK4积分器、一维/二维插值等纯数学运算。
2.  **算法库层 (Algorithm Library)**：与飞行器无关的 GNC 通用算法，如增量 PID、巴特沃斯滤波器、坐标静态转换公式等。
3.  **接口层 (Interface Layer)**：定义组件间通信的纯虚接口（如 `IDynamicsModel`, `IGuidance3DOF`）。这是解耦的核心。
4.  **服务层 (Service Layer)**：跨组件的全局设施。通过 `ServiceContext` 按需注入。例如使用树形拓扑与 LCA 算法管理的 `CoordinateService`。
5.  **应用层 (Application Layer)**：包含 `Simulator` 核心调度引擎和各种具体的组件实现。

---

## 2. 核心引擎微设计 (Micro-designs)

真正的工程挑战往往隐藏在细节中。以下是本框架在引擎层面的核心微设计：

### 2.1 零依赖的配置管理器 (`ConfigManager` & `ConfigNode`)
作为一个轻量级 C++ 框架，引入大型第三方 JSON 库（如 nlohmann/json）往往会让编译变得臃肿。
*   **微设计**：框架内部手写了一个极其精简的递归下降解析器 `JsonParser` 和 `ConfigNode` 树。
*   **访问追踪 (Access Tracking)**：`ConfigNode` 内置了 `accessed_keys_` 集合。当组件读取配置时（如 `config["mass"]`），该 key 会被标记。这使得框架能够检测出 JSON 中**未被使用的冗余配置项**，并向用户发出警告。

### 2.2 状态向量的内存布局 (`StateLayout`)
在动力学积分中，状态（位置、速度、姿态等）必须被展平为一个一维的 `Eigen::VectorXd` 才能交给 RK4 积分器。
*   **微设计**：`StateLayout` 提供了一个双向映射字典。组件可以通过字符串名字（如 `"altitude"`）动态申请状态位，并获得一个整型索引。这使得动力学组件能够灵活地支持 3DOF（6维状态）到 6DOF（13维状态）的无缝切换，而不需要硬编码状态数组的下标。

### 2.3 依赖验证防御网 (`DependencyValidator`)
在组件实例化后、仿真正式开始前，框架会执行严格的依赖图扫描：
*   **机制**：组件可以继承 `IDependencyDeclarer`，返回其需要的 `interface_type` 及其是否是必选项（`required`）。
*   **验证**：`DependencyValidator::validate()` 会扫描全局注册表。如果某组件声明了强依赖 `IDynamicsModel`，但当前 JSON 没配置，验证器会**直接阻断仿真启动**，并打印极具指导意义的错误信息。

### 2.4 隔离的注册表视图 (`ScopedRegistry`)
为了支持多飞行器对抗（如导弹打飞机），不同飞行器的同名组件（如 `missile.dynamics` 和 `target.dynamics`）在全局注册表 `ComponentRegistry` 中是带前缀的。
*   **微设计**：引擎会为每个组件创建一个 `ScopedRegistry`。当导弹的制导组件调用 `registry.getByName<IDynamicsModel>("dynamics")` 时，`ScopedRegistry` 会自动补全前缀为 `"missile.dynamics"`。如果组件名包含 `.`（如 `"env.earth"`），则视为绝对路径进行全局查找。这种设计让多实体仿真无需修改任何组件内部代码！

### 2.5 按需付费的服务注入 (`ServiceContext`)
与基于名称字符串查找的 `ComponentRegistry` 不同，服务（如坐标转换服务）属于全局基础设施。
*   **微设计**：`ServiceContext` 是一个基于 `std::type_index` 的强类型异构容器。组件通过 `services.get<CoordinateService>()` 直接获取类型安全的单例指针。这体现了 C++ "Pay only for what you use" 的原则，没用到的服务不会被实例化。

### 2.6 积分器拦截与高频调度
在 `Simulator::step()` 的源码中，有一个极其重要的细节：
```cpp
auto* dyn_model = dynamic_cast<interfaces::IDynamicsModel*>(component);
if (dyn_model && integrator_) {
    // 使用 RK4 等专门的数值积分器步进
    integrator_->step(..., current_time_, x, config_.dt);
} else {
    // 普通组件按自己的频率 update
    component->update(config_.dt);
}
```
*   **微设计**：动力学模型并不是通过普通的 `update()` 进行更新的！引擎会使用 `dynamic_cast` 探测组件是否为动力学模型。如果是，它将被交由专用的数值积分器（如 `RK4Integrator`）进行多步导数计算（`computeDerivatives`）。这保证了物理运动方程的高精度求解，而普通算法组件依然使用廉价的单步 `update`。

---
> 了解了引擎底层的微设计后，请前往 [**数据流向与工作流程机制**](DataFlow_Workflow.md) 看看数据是如何在这些机制之上流转的。
