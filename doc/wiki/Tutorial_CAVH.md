# 深入实战教程：利用框架特性解决复杂业务挑战

仅仅展示怎么写代码是远远不够的，真正的工程能力体现在“如何优雅地解决业务痛点”。
本教程以 `03_cavh_3dof` (CAV-H 高超声速滑翔飞行器 3DOF 仿真) 为蓝本，抛弃枯燥的代码堆砌，直接带您领略框架的**配置解析追踪、状态动态寻址、依赖验证网、以及无侵入状态观测**是如何在实际开发中大显身手的。

## 业务挑战 1：气动系数依赖高度非线性查表，如何保证时序正确？

在高超声速飞行中，气动组件（`CavhAerodynamics`）需要实时的**攻角**（来自 Guidance）和**马赫数**（需要从 Dynamics 的速度除以声速获得）。

**错误做法**：在 `update` 里写死指针查找，一旦别人在 JSON 中删了 Guidance 组件，直接报空指针异常。

**框架级解法：依赖声明防御网 (`IDependencyDeclarer`)**

```cpp
#include "gnc/core/dependency_validator.hpp"

class CavhAerodynamics : public gnc::core::ComponentBase,
                         public gnc::interfaces::IAeroCoefficients,
                         public gnc::core::IDependencyDeclarer { // 1. 继承声明器接口
// ...
    // 2. 向引擎明确声明：我必须依赖这俩，否则不要启动仿真！
    std::vector<gnc::core::DependencyDeclaration> getDependencies() const override {
        return {
            {std::type_index(typeid(gnc::interfaces::IGuidance3DOF)), "3DOF Guidance", true},
            {std::type_index(typeid(gnc::interfaces::IDynamicsModel)), "Dynamics Model", true}
        };
    }

    // 3. 在这安全地拿指针（引擎验证通过后才会调这里）
    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        guidance_ = registry.getByName<gnc::interfaces::IGuidance3DOF>("guidance");
        dynamics_ = registry.getByName<gnc::interfaces::IDynamicsModel>("dynamics");
    }
};
```
如果你在 JSON 中拼写错了 `"dynamics"`，引擎会在 `Initializing` 阶段直接阻断，并提示：`CavhAerodynamics requires Dynamics Model ... but no registered component provides this interface.` 

---

## 业务挑战 2：动力学组件如何处理多变的状态维度？

如果今天我们要写一个 3DOF（状态是[经度, 纬度, 高度, 速度, 弹道倾角, 偏航角]），明天我们要写 6DOF，如果硬编码 `state[2]` 代表高度，代码将无法维护。

**框架级解法：状态动态寻址 (`StateLayout`)**

在你的动力学组件中，利用框架的 `StateLayout`：

```cpp
// 在动力学组件的初始化中：
layout_.addVariable("longitude");
layout_.addVariable("latitude");
layout_.addVariable("altitude");
// ...

// 任何其他组件想要拉取高度，不需要知道数组下标：
double alt = dynamics_->getStateValue("altitude");
```
这正是 `CavhProgrammedAoA`（程序攻角制导组件）能够根据高度动态计算期望攻角的底层基石！

---

## 业务挑战 3：JSON 配置写错或写多了怎么办？

随着仿真变复杂，`config` 块可能堆积了几十个参数。如果拼写错误（比如把 `cl_alpha` 写成了 `cl_alfa`），传统的 C++ 解析器可能直接用默认值，导致诡异的仿真结果。

**框架级解法：配置访问追踪 (`ConfigNode::accessed_keys_`)**

框架手写的 `ConfigNode` 是自带“探针”的。当你在 `configure` 中读取：
```cpp
void configure(const gnc::core::ConfigNode& config) override {
    cl_alpha_ = config["cl_alpha_per_rad"].asDouble(1.6);
}
```
`ConfigNode` 内部会将 `"cl_alpha_per_rad"` 标记为已访问。
在仿真启动前，框架会扫描 JSON，并找出所有**未被 C++ 代码读取的 Key**。如果发现 `"cl_alfa": 1.6` 没人读，引擎会打印一条 WARNING，直接帮你揪出拼写错误！

---

## 业务挑战 4：如何无侵入地记录中间变量？

我们需要导出“升阻比”，但这并不是框架预定义的数据结构。我们不想为了记录数据去改动 Logger 的代码。

**框架级解法：动态观测器注入 (`IObservable`)**

```cpp
#include "gnc/interfaces/infrastructure/i_observable.hpp"

class CavhAerodynamics : public ..., public gnc::interfaces::IObservable {
// ...
    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        
        // 暴露 CL 和 CD
        builder.addScalar("CL", [this]() { return currentCoefficients().CL; });
        builder.addScalar("CD", [this]() { return currentCoefficients().CD; });
        
        // 暴露自定义计算的升阻比！
        builder.addScalar("lift_to_drag", [this]() {
            const auto coeffs = currentCoefficients();
            return std::abs(coeffs.CD) > 1e-9 ? coeffs.CL / coeffs.CD : 0.0;
        });
        
        return builder.build();
    }
};
```
在 JSON 中，你只需告诉引擎：
```json
"outputs": {
    "record": {
        "aero": ["CL", "CD", "lift_to_drag"] 
    }
}
```
`AutoDataLogger` 会在每步结束时，通过回调你绑定的 Lambda 表达式抓取这些值，完美解耦！

---

## 业务挑战 5：仿真条件复杂，如何动态终止？

让 CAV-H 在高度低于 10km 时停止。传统的做法是在主循环写死 `if (h < 10000) break;`。
在 GNC 框架中，控制权完全在 JSON 手里：

```json
"stop_conditions": [
    {
        "type": "component_field_below",
        "component": "dynamics",
        "field": "altitude",
        "value": 10000.0
    }
]
```
引擎会在解析时，利用 `Registry` 拿到 `dynamics` 的指针，并利用 `StateLayout` 构建一个判断高度是否低于 10000 的闭包函数，注入到 `Simulator` 的停止条件列队中。

**总结**：这就是 GNC 仿真框架的设计哲学。不要用 C++ 去写死流程控制，利用好 `IDependencyDeclarer`、`IObservable` 和 JSON 配置树，让框架的底层引擎去为你做这些苦力活！
