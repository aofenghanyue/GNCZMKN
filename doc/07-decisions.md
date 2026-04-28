# 设计决策

## ADR-001: Mission 使用 `vehicles[]`

当前有效 mission schema 使用顶层 `vehicles[]`。单飞行器任务也是 `vehicles` 中一个条目。

决定：

- 不支持旧式 `entities[]`。
- 不支持根级 `components/services`。
- 不支持根级 `form/vehicle/interaction`。
- 每个 vehicle 条目拥有独立 `<id>.` 作用域。

## ADR-002: 周期开始发布态

固定步长循环以周期开始发布态为唯一记录和停止条件时间点：

```text
publish/refresh t_k -> before_step(t_k) -> update(t_k)
-> record t_k -> stop check t_k
-> synchronized independent integration to t_{k+1}
```

原因：

- CSV `time` 与状态字段严格对应。
- form/dynamics/truth view 字段是周期开始发布态 `x_k` 及其真实派生量。
- input/process/guidance/output 字段是 record 时刻组件暴露的本周期离散输出，因为 record 发生在 `update(t_k)` 之后。
- `t0` 行包含初始状态 `x_0` 和基于 `x_0` 计算出的第一周期离散输出。
- 框架不保证所有 observable 在物理意义上无延迟；延迟语义由组件模型自身定义。
- form `FlightStateView` 是 truth/form 层真实飞行状态视图；机上导航或估计飞行状态应由 `vehicle.process` 组件产生。
- 触发停止条件的状态会保留在最后一行。
- GNC 离散算法读取的是已发布的采样状态。

## ADR-003: 同步发布点的独立连续积分

当前不实现全局联合 ODE。连续系统按组件独立积分，但每步统一提交 next state。

这保留了轻量组件式结构，同时避免同一步内其他连续组件先提交导致导数读取到部分新状态。未来若需要强耦合，应新增明确的 coupled group。

## ADR-004: `update()` 是离散入口

`update()` 保留为统一离散入口。它读取 t_k 发布态，产生 `[t_k, t_{k+1}]` 的本周期保持输出，不负责连续积分。连续刷新使用 `publish()`。

## ADR-005: Interaction 是高级 closure

`interaction` 保留为高级扩展点，但普通用户应优先配置内置 interaction。气动、质量、推进模型属于 `vehicle.output`；interaction 只做组合和闭合，并且必须声明 form family。

## ADR-006: 配置严格化

builtin 使用严格配置读取：

- 物理参数 required，不静默默认。
- unknown key 是 build error。
- 类型错误是 build error。
- 错误消息包含 mission 路径。

安全默认值仍可保留，例如 `integrator = rk4`。

## ADR-007: 多飞行器 snapshot 后置

本阶段只文档化跨 vehicle 读取应理解为发布态读取，不实现 world snapshot API，也不禁止现有 direct lookup。snapshot API 和 direct lookup 限制作为第二阶段单独设计。
