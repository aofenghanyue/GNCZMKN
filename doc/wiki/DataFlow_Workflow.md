# 数据流向与工作流程机制

本篇将彻底揭开 GNC 仿真框架运转的底层秘密：**零拷贝的拉取机制、多态积分器拦截，以及条件驱动的仿真终止。**

## 1. 摒弃 Message 总线：零拷贝的 Pull 模型

传统的 ROS 仿真框架中，节点间通过 Pub/Sub 交换 Message。这在 GNC 仿真中有两大原罪：
1.  **内存拷贝开销**：高频仿真下，序列化和拷贝带来极大的 CPU 负担。
2.  **异步时序混乱**：控制算法很难保证拿到的是“当前时间步”的最新动力学状态，还是上一步的滞后状态。

**GNC 框架的破局之道：**
采用**接口查询 + 主动拉取 (Interface Query + Pull)**。

### 1.1 数据流向与时序强绑定
数据的流向完全由 JSON 配置文件中 `components` 数组的顺序决定。
例如，典型的时序为：`Earth` $\rightarrow$ `Mass` $\rightarrow$ `Aerodynamics` $\rightarrow$ `Guidance` $\rightarrow$ `Dynamics`。

当 `Aerodynamics::update()` 被调用时，它需要攻角和马赫数。它通过 `guidance_` 和 `dynamics_` 指针**直接调用函数**：
```cpp
const double alpha = guidance_->getFlightCommand().alpha;
const double mach = dynamics_->getStateValue("velocity") / 340.0;
```
没有中间结构体，没有队列，这是绝对的 **Zero-Copy**。

---

## 2. 仿真主循环的深度解剖 (`Simulator::run`)

主循环并不是简单的 `for` 循环遍历组件。它隐藏了精妙的拦截逻辑。

### 2.1 积分器拦截机制 (Integrator Interception)
在 `Simulator::step()` 中，引擎会对组件进行类型甄别 (Type-sniffing)：
```cpp
auto* dyn_model = dynamic_cast<interfaces::IDynamicsModel*>(component);
if (dyn_model && integrator_) {
    // 它是动力学模型！拦截其常规 update，交由 RK4 积分器处理
    integrator_->step( [dyn_model](t, state, dxdt){ dyn_model->computeDerivatives(...); }, ... );
} else {
    // 常规算法组件
    component->update(config_.dt);
}
```
**设计意义**：这意味着你可以随时在 JSON 中把 `"integrator": "rk4"` 换成 `"euler"` 或 `"rk45"`，而不需要修改动力学组件的任何代码！动力学组件只负责提供导数（`computeDerivatives`），推进状态的工作由引擎接管。

### 2.2 数据采集阶段 (`AutoDataLogger`)
在所有组件的 `step()` 或 `update()` 结束后，主循环会统一调用 `auto_logger_.recordStep(current_time_)`。
此时，Logger 会遍历所有注册了 `IObservable` 的组件，调用我们在 `ObservableFieldBuilder` 中绑定的 Lambda 表达式（如 `[this](){ return mass_; }`），将所有变量抓取并刷入 CSV 文件。
这也是为什么闭包中捕获 `[this]` 指针是安全的，因为 Logger 的调用发生在该时间步的末尾。

### 2.3 仿真终止条件 (`TerminationCondition`)
仿真何时结束？不仅仅是时间到了 `duration`。
框架支持注入动态终止条件。在主循环的最后：
```cpp
for (const auto& condition : termination_conditions_) {
    if (condition.condition(step, current_time_)) {
        termination_reason_ = condition.name;
        goto simulation_end; // 提前安全终止
    }
}
```
在 JSON 配置中，你可以轻松配置：
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
引擎会在解析 JSON 时，自动将上述逻辑转化为一个 Lambda 表达式注入到 `termination_conditions_` 中。这极大地释放了配置文件的能力！
