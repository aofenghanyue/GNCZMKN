# 自定义组件开发指南

在 GNC 仿真框架中，所有的业务逻辑（从气动查表到复杂的 PID 控制）都必须封装成一个**组件（Component）**。本指南将带您从头手写一个标准组件，并解析生命周期中的每一个关键步骤。

## 1. 组件的三大核心要素

一个合格的框架组件必须具备以下特征：
1.  **继承基类**：必须 public 继承 `gnc::core::ComponentBase`。
2.  **实现契约接口**：必须继承至少一个 `gnc::interfaces::IXXX` 接口（如果是纯粹的计算组件，也可以不继承，但通常没有意义）。
3.  **注册到工厂**：在 `.cpp` 或 `.hpp` 的末尾，使用宏 `GNC_REGISTER_COMPONENT` 进行注册。

---

## 2. 组件基类 `ComponentBase` 的生命周期

在 `simulator.run()` 执行期间，组件的各个方法会按特定顺序被调用。这是您编写代码的核心舞台：

### 阶段一：构建与配置 (Build & Configure)
```cpp
virtual void configure(const gnc::core::ConfigNode& config);
```
**作用**：在解析 JSON 配置文件时被调用。
**用法**：从 `config` 中读取您在 JSON 中配置的参数（如 `mass`, `kp`, `ki` 等）。内置了强大的类型转换如 `config["mass"].asDouble(100.0)`。

### 阶段二：依赖注入 (Dependency Injection)
```cpp
virtual void injectDependencies(gnc::core::ScopedRegistry& registry);
virtual std::vector<DependencyDeclaration> getDependencies() const; // (可选，如果继承了 IDependencyDeclarer)
```
**作用**：在所有组件实例化完成后，互相建立联系。
**用法**：使用 `registry.getByName<T>("name")` 获取其他组件的接口指针，并保存为成员变量。

### 阶段三：初始化 (Initialize)
```cpp
virtual void initialize();
```
**作用**：在仿真第一步（$t=0$）前调用。
**用法**：初始化内部状态（如积分器的历史误差清零，滤波器状态复位等）。

### 阶段四：仿真主循环更新 (Update)
```cpp
virtual void update(double dt);
```
**作用**：这是**最核心的算法执行位置**。Simulator 会根据当前组件设置的执行频率（`setExecutionFrequency(Hz)`），决定是否在当前时间步调用此方法。
**用法**：通过注入的接口指针拉取其他组件的数据 $\rightarrow$ 执行您的算法逻辑 $\rightarrow$ 将结果暂存在成员变量中，供其他组件随后拉取。

### 阶段五：清理 (Finalize)
```cpp
virtual void finalize();
```
**作用**：仿真结束时调用。通常无需重写。

---

## 3. 示例：开发一个“恒定推力”制导组件

我们来编写一个极其简单的组件：它不看导航状态，永远输出恒定的 Z 轴加速度指令。

**第一步：创建头文件 `my_constant_guidance.hpp`，放在 `user/components/` 下。**

```cpp
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/gnc/i_guidance.hpp"

// 1. 继承 ComponentBase 和 IGuidance 接口
class MyConstantGuidance : public gnc::core::ComponentBase, 
                           public gnc::interfaces::IGuidance {
public:
    // 2. 构造函数中必须调用 ComponentBase 的构造函数，并传入组件名称
    MyConstantGuidance() : ComponentBase("MyConstantGuidance") {
        // 设置执行频率为 50Hz
        setExecutionFrequency(50.0);
    }

    // 3. 实现 configure 方法，读取 JSON 配置
    void configure(const gnc::core::ConfigNode& config) override {
        // 尝试读取 "thrust_z"，如果 JSON 中没有，则默认值为 10.0
        thrust_z_ = config["thrust_z"].asDouble(10.0);
    }

    // 4. 实现 update 方法，执行核心逻辑
    void update(double dt) override {
        // 由于是恒定制导，每步都输出固定的 Z 轴加速度
        cmd_.acceleration_cmd = {0.0, 0.0, thrust_z_};
        cmd_.attitude_cmd = {1.0, 0.0, 0.0, 0.0}; // 零姿态角 (w, x, y, z)
        cmd_.timestamp = getSimTime(); // getSimTime() 是基类提供的方法
    }

    // 5. 实现 IGuidance 接口强制要求的方法，供控制器(Controller)拉取
    const gnc::interfaces::GuidanceCommand& getGuidanceCommand() const override {
        return cmd_;
    }
    
    void setTarget(const gnc::Vector3d& target) override {
        // 本示例不需要目标点
    }
    
    bool isActive() const override { return true; }

private:
    double thrust_z_ = 0.0;
    gnc::interfaces::GuidanceCommand cmd_; // 缓存指令结果
};

// 6. 最关键的一步：将组件注册到工厂
// 第一个参数是类名，第二个参数是它对外暴露的接口类型
GNC_REGISTER_COMPONENT(MyConstantGuidance, gnc::interfaces::IGuidance)
```

## 4. 暴露变量给 CSV 记录器 (`IObservable`)

如果希望将 `thrust_z_` 变量输出到 CSV 文件中，您需要继承 `IObservable` 接口：

```cpp
#include "gnc/interfaces/infrastructure/i_observable.hpp"
#include "gnc/core/observable_helpers.hpp"

class MyConstantGuidance : public gnc::core::ComponentBase, 
                           public gnc::interfaces::IGuidance,
                           public gnc::interfaces::IObservable { // <-- 新增继承
// ...
public:
    // 实现 getObservableFields 方法
    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        // 绑定一个 Lambda 表达式。注意捕获 [this]
        builder.addScalar("current_thrust_z", [this]() { return thrust_z_; });
        return builder.build();
    }
// ...
};
```
在 JSON 的 `outputs.record.guidance` 中配置 `["current_thrust_z"]` 后，该变量就会被自动记录！

> 下一步：组件编写完成后，它是如何被调度的？请阅读 [**数据流向与工作流程**](DataFlow_Workflow.md) 或直接前往实战 [**基于 CAV-H 的新手教程**](Tutorial_CAVH.md)。
