# 深入实战教程：基于 CAV-H 掌握框架高级特性

仅仅让代码跑起来是不够的。本教程将以 `03_cavh_3dof` 为蓝本，抛弃枯燥的代码堆砌，直接带您领略框架的**配置解析机制、依赖验证网、以及动态状态观测**是如何在实际开发中大显身手的。

## 1. 案例需求拆解

**CAV-H** (高超声速滑翔飞行器) 3DOF 下滑仿真。
**挑战**：
1. 气动系数（CL, CD）是马赫数和攻角的非线性函数。这意味着气动组件必须从其他组件**拉取**数据。
2. 期望的攻角是基于当前高度分段插值的。制导组件必须**拉取**高度。
3. 需要在高度低于 10km 时自动停止仿真。
4. 需要将飞行过程中的“升阻比 (Lift-to-Drag)”和“攻角”输出为图表。

---

## 2. 攻克挑战 1 & 2：安全的依赖注入

在编写 `CavhAerodynamics` 时，我们需要攻角（来自 Guidance）和马赫数（来自 Dynamics）。
**错误做法**：在 `update` 里直接去找，找不到就崩溃。
**正确做法**：利用框架的微设计——`IDependencyDeclarer`！

```cpp
#include "gnc/core/dependency_validator.hpp"

class CavhAerodynamics : public gnc::core::ComponentBase,
                         public gnc::interfaces::IAeroCoefficients,
                         public gnc::core::IDependencyDeclarer { // 1. 继承声明器
// ...
    // 2. 向框架注册：我必须要有这俩接口，否则别启动！
    std::vector<gnc::core::DependencyDeclaration> getDependencies() const override {
        return {
            {std::type_index(typeid(gnc::interfaces::IGuidance3DOF)), "3DOF Guidance", true},
            {std::type_index(typeid(gnc::interfaces::IDynamicsModel)), "Dynamics Model", true}
        };
    }

    // 3. 在这里安全地拿指针
    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        guidance_ = registry.getByName<gnc::interfaces::IGuidance3DOF>("guidance");
        dynamics_ = registry.getByName<gnc::interfaces::IDynamicsModel>("dynamics");
    }
};
```
有了这段代码，如果我们在 JSON 中忘记配置 `"dynamics"`，引擎的 `DependencyValidator` 会在初始化阶段拦截，并打印：`"CavhAerodynamics requires Dynamics Model ... but no registered component provides this interface."` 这能节省你几个小时的 Debug 时间！

---

## 3. 攻克挑战 4：利用 IObservable 暴露计算结果

我们需要导出“升阻比”，但这并不是框架内置的属性。我们需要在 `CavhAerodynamics` 中计算并暴露它。

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

接着，在 `cavh_mission.json` 中配置日志引擎：
```json
"outputs": {
    "record": {
        "aero": ["CL", "CD", "lift_to_drag"]  // 名字必须与 addScalar 中的一致！
    }
}
```
引擎的 `AutoDataLogger` 会在每步结束时，通过回调 Lambda 提取并记录这些值！

---

## 4. 攻克挑战 3：JSON 驱动的高级仿真控制

我们不需要在 C++ 代码里写 `if (altitude < 10000) exit(0);`，这违反了配置驱动的原则。
在 `cavh_mission.json` 中，我们可以直接利用 `Simulator` 提供的动态终止条件解析能力：

```json
{
    "simulation": {
        "dt": 0.1,
        "duration": 600.0,
        "stop_conditions": [
            {
                "type": "component_field_below",
                "component": "dynamics",
                "field": "altitude",
                "value": 10000.0
            }
        ]
    }
}
```
底层原理：`SimulatorBuilder` 在解析时，会利用反射获取 `dynamics` 组件的指针，然后绑定一个比较 `getStateValue("altitude") < 10000.0` 的 Lambda 表达式到主循环中！

---

## 5. 总结：这套组合拳的威力

通过上述设计，我们将 CAV-H 的仿真剥离成了：
1.  **极度纯粹的物理组件**：气动只算气动，制导只算插值，代码极短。
2.  **绝对安全的依赖网**：由 `DependencyValidator` 在启动前把关。
3.  **零侵入的日志**：通过 `IObservable` 和 Lambda 闭包，按需导出。
4.  **外置的流程控制**：积分器类型、终止条件、执行频率全部交由 JSON 和 `Simulator` 接管。

这才是 GNC 仿真框架设计的最高境界：**让算法工程师写最少的代码，得到最健壮、最可视化的结果！**
