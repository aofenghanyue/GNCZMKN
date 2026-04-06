# 接口设计与组件进阶开发规范

编写一个能运行的组件很容易，但要编写一个**安全、可观测、多频率友好**的工业级组件，必须深入理解框架的进阶规范。

## 1. 组件开发的三重境界

### 1.1 第一重：满足基本生命周期
继承 `ComponentBase`，在 `configure` 中读取 JSON，在 `update(dt)` 中计算并存储状态。

### 1.2 第二重：安全地声明依赖 (`IDependencyDeclarer`)
不要仅在 `injectDependencies` 中默默地调用 `getByName` 并在运行时崩溃。您应当主动声明依赖：

```cpp
#include "gnc/core/dependency_validator.hpp"

class MyComponent : public gnc::core::ComponentBase, 
                    public gnc::core::IDependencyDeclarer { // 继承声明接口
// ...
    // 明确告诉框架你需要什么
    std::vector<gnc::core::DependencyDeclaration> getDependencies() const override {
        return {
            {std::type_index(typeid(gnc::interfaces::IDynamicsModel)), "Dynamics Model (Required)", true},
            {std::type_index(typeid(gnc::interfaces::IAtmosphereModel)), "Atmosphere (Optional)", false}
        };
    }
// ...
};
```
这样，如果用户在 JSON 中漏配了组件，框架会在 `Initializing` 阶段优雅地抛出错误，而不是在 `Running` 阶段发生空指针段错误（Segfault）。

### 1.3 第三重：实现无缝日志观测 (`IObservable`)
如何让自己的内部变量能够被 CSV 记录下来？不需要手动写文件！框架提供了 `ObservableFieldBuilder` 这一绝佳的微设计。

```cpp
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/core/observable_helpers.hpp"

class MyGuidance : public ..., public gnc::interfaces::IObservable {
// ...
    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        
        // 添加标量：在 JSON 的 record 中配 "alpha_deg" 即可记录
        builder.addScalar("alpha_deg", [this]() { return command_.alpha * 57.3; });
        
        // 自动展开三维向量：配置 "target_pos"，框架会自动记录 target_pos.x, y, z
        builder.addVector3d("target_pos", [this]() -> const gnc::Vector3d& { return target_; });
        
        return builder.build();
    }
};
```
注意闭包捕获：必须通过 `[this]` 捕获对象自身，因为 `AutoDataLogger` 会在仿真循环结束时统一回调这些 Getter 拉取数据。

---

## 2. 多频率调度的秘密 (`Execution Frequency`)

在构造函数中，您通常会调用 `setExecutionFrequency(20.0)`。
在底层，`Simulator::computeStepIntervals()` 会计算出您的组件需要跳过多少个基础步长：
`interval = round( sim_freq / comp_freq )`

**最佳实践与注意点**：
1.  **基础 dt 是王道**：如果在 JSON 中配置 `dt: 0.1` (10Hz)，但你在代码中 `setExecutionFrequency(100.0)`，那么 interval = 1。你的组件实际运行频率是被截断的 10Hz，不可能比基础步长更快。
2.  **如何判断是否在执行？**：框架内部会调用 `shouldExecute(step)`。如果您需要在组件外部知道某一步是否是您的控制周期，可以调用这个方法。
3.  **dt 的传递**：在您的 `update(double dt)` 中，传入的 `dt` 是**基础仿真步长**，**不是**您的控制周期！如果您的 PID 控制器需要使用实际的控制时间间隔，应当自己乘以 interval，或者使用 `dt * getStepInterval()`。

> 下一步：想看这些高级特性在真实代码中如何组合？请阅读 [**基于 CAV-H 的新手教程**](Tutorial_CAVH.md)。
