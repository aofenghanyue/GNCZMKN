# 维护者架构说明

本文面向 framework 维护者，说明当前装配、调度、日志、停止条件和配置校验边界。

## 构建流程

1. `runner.cpp` 注册 builtin packages 和 active project components。
2. `SimulationBuilder` 加载 mission，严格解析 `simulation` 和 integrator。
3. `MissionAssembler` 装配 environment、vehicles、services 和 components。
4. `ValidationPipeline` 检查 role/stage/form-family 和依赖。
5. `StopConditionBuilder` 绑定停止条件；错误作为 build error。
6. `AutoDataLogger` 根据 `outputs` 初始化记录字段。

## 调度模型

`Simulator` 使用固定步长和周期开始发布态：

```text
publish/refresh t_k
before_step(step, t_k, dt)
discrete update(t_k)
record t_k
stop check t_k
synchronized independent integration to t_{k+1}
after_step(step, t_{k+1}, dt)
```

`after_step` 在下一次 publish 之前，因此 truth/view 仍表示上一发布态；它是维护者低层钩子，不应成为用户记录或停止条件接口。

publish 先使用轻量 phase 顺序，再使用组件 priority：

```text
state owners（IContinuousSystem 或显式 StateOwner）
-> views / observers
-> ordinary components
```

在同一 execution stage 或 publish phase 内，较小的 `priority` 先执行；priority 相同则保持 mission/config 注册顺序。`priority` 必须是整数式 number。默认调度器不会让 priority 把 view 排到 state owner 前面。

## 连续系统

`IContinuousSystem` 不在 stage 中直接提交状态。每周期末先计算所有 next state，再统一 `setState()`：

- 导数读取的是 t_k 发布态。
- 提交发生在所有连续系统 next state 都算完之后。
- 这不是全局联合 ODE，也不表达连续强耦合。

如果未来需要联合积分，应新增明确的 state assembler 或 coupled group。

## 配置校验

`ConfigReader` 是 builtin 的轻量配置读取辅助。原则：

- 安全默认值可以保留，例如 `integrator = rk4`。
- 物理参数不静默默认，例如初始状态、质量、气动表、控制表。
- builtin unknown key 是 build error。
- 类型错误和 required 缺失是 build error。
- 错误消息应包含路径，例如 `vehicles[0].form.components[0].config.initial_state.altitude_m`。

`simulation.dt > 0`，`duration >= 0`，且 `duration / dt` 必须为整数步数。

## 日志和停止条件

`AutoDataLogger` 记录 `IObservable` 字段。CSV 每行对应周期开始发布态 `t_k`；默认记录 `t0`。`record_initial_state=false` 可跳过 t0 数据行，但停止条件仍基于发布态。
CSV 行在 `update(t_k)` 之后写出：form/dynamics/truth view 字段是周期开始发布态 `x_k` 及其真实派生量；input/process/guidance/output 字段是 record 时刻各组件暴露的本周期离散输出。`t0` 行包含初始状态 `x_0` 和基于 `x_0` 计算出的第一周期导航/制导/输出。框架不保证所有 observable 在物理意义上无延迟；延迟语义由组件模型自身定义。

`FlightStateView` 当前指 form/truth 层真实飞行状态视图，在 publish 阶段刷新。若需要机上导航或估计飞行状态，应建模为 `vehicle.process` 组件，而不是复用 form truth view。

停止条件使用相同发布态，并在 record 之后检查，因此触发状态默认保留在 CSV 最后一行。

## 多飞行器

当前多飞行器不是连续强耦合系统。文档规定跨 vehicle 读取应理解为读取发布态；world snapshot API 留到后续阶段实现。本阶段不禁止现有 direct lookup，避免把时间语义和跨 vehicle API 改造绑在同一风险面。
