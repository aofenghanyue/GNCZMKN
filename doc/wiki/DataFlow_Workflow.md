# 数据流向与工作流程机制

本篇将揭示 GNC 仿真框架运转的底层秘密：**数据是如何在组件间流动的？主循环是如何调度的？多频率的组件是如何协调的？**

## 1. “接口查询 + 主动拉取”机制取代消息总线

许多仿真框架（如 ROS）采用**发布/订阅（Pub/Sub）机制**：组件计算完数据后，将数据打包成 Message 丢进总线，其他组件通过回调函数（Callback）接收。

**GNC 框架的痛点与革新：**
在 GNC 仿真中，控制频率通常极高（100Hz ~ 1000Hz），如果采用 Pub/Sub，不仅会带来巨大的序列化/反序列化和**内存拷贝开销**，还会导致极其难以调试的“异步时序错乱”问题。

因此，本框架**彻底摒弃了消息总线**，采用**接口查询 + 主动拉取 (Interface Query + Pull)** 的零拷贝模式。

### 1.1 接口查询 (Dependency Injection)
组件在初始化（`injectDependencies`）时，通过 `ScopedRegistry` 获取所需组件的**指针**。
```cpp
void injectDependencies(gnc::core::ScopedRegistry& registry) override {
    // 获取“导航”组件的指针，类型是纯虚接口 INavigation*
    nav_ = registry.getByName<gnc::interfaces::INavigation>("nav");
}
```

### 1.2 主动拉取 (Pull Model)
在每个仿真步（`update(dt)`），组件通过刚才拿到的指针，直接调用接口方法拉取数据。数据通常以 `const &` 返回，**零拷贝**。
```cpp
void update(double dt) override {
    // 直接从内存中拉取导航组件最新算好的状态
    const auto& state = nav_->getNavState(); 
    // ... 执行当前组件的计算 ...
}
```

---

## 2. 仿真主循环 (Main Loop) 与时序控制

仿真器（`Simulator`）接管了所有的调度工作。一个完整的仿真循环可以概括为以下伪代码：

```cpp
void Simulator::run() {
    double t = 0.0;
    int step = 0;

    // 1. 初始化所有组件
    for (auto* comp : components_) {
        comp->initialize();
    }

    // 2. 主循环
    while (t < duration_ && !checkStopConditions()) {
        
        // 3. 遍历所有组件
        for (auto* comp : components_) {
            // 4. 检查该组件在当前时间步是否需要执行 (频率控制)
            if (comp->shouldExecute(step)) {
                comp->update(dt_); // 核心：执行组件的业务逻辑
            }
        }
        
        // 5. 记录日志 (如果该步需要记录)
        data_logger_->logStep(t);
        
        // 6. 推进时间
        t += dt_;
        step++;
    }

    // 7. 清理
    for (auto* comp : components_) {
        comp->finalize();
    }
}
```

### 2.1 依赖与执行顺序 (Execution Order)

如果 `Controller` 拉取 `Guidance` 的数据，显然 `Guidance` 必须先执行计算（`update`）。
**框架是如何保证时序的？**

答案是：**完全由 JSON 配置文件中组件数组的顺序决定。**
Simulator 会严格按照 JSON 列表中 `components` 数组的声明顺序来执行每个组件的 `update()`。

**标准（推荐）的 GNC 时序：**
1.  **环境模型** (`Earth`, `Atmosphere`, `Gravity`)：提供自然环境底座。
2.  **飞行器特性** (`Mass`, `AeroCoefficients`)：质量和气动参数（可能依赖环境或动力学反馈的马赫数）。
3.  **动力学积分器** (`Dynamics`)：利用上一步的力/力矩，积分出当前步的真实状态（位置、速度、姿态）。
4.  **传感器** (`IMU`, `GPS`)：拉取真实状态，加入噪声。
5.  **导航** (`Navigation`)：拉取传感器数据，进行滤波估计。
6.  **制导** (`Guidance`)：拉取导航状态，计算期望指令。
7.  **控制** (`Controller`)：拉取制导指令和导航状态，计算执行机构偏角/推力，**最后将力/力矩推送给 `Dynamics` 的受力累加器中**，供下个周期积分。

---

## 3. 多频率组件协调 (Multi-Rate Scheduling)

实际工程中，传感器的采样率、控制器的解算率和动力学的积分率通常是不同的。
- 动力学积分：1000 Hz
- IMU 传感器：100 Hz
- GPS 传感器：10 Hz
- 制导计算：50 Hz

**在 JSON 中，我们只需要配置基础仿真步长 `dt` (如 0.001s, 即 1000Hz)。**
各个组件通过 `setExecutionFrequency(Hz)` 或者 JSON 的 `config: { "frequency_hz": 100 }` 来设置自己的运行频率。

`Simulator` 在初始化时，会自动计算出每个组件的**执行间隔（Interval）**。
例如：基础 dt = 0.001s (1000Hz)。
*   IMU 设置了 100Hz。Simulator 会计算出它的 Interval = 1000 / 100 = 10 步。
*   在主循环的 `shouldExecute(step)` 判断中，IMU 只会在 `step % 10 == 0` 时返回 `true`，从而被执行。其余 9 步直接跳过，节省算力！

---
> 下一步：通过实战案例来验证这些概念，请前往 [**基于 CAV-H 的新手全流程教程**](Tutorial_CAVH.md)。
