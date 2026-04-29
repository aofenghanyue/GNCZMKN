# 设计决策

## ADR-008: P2 Form Extension Contract

New form families must publish an explicit contract before they are used in a mission. The minimum contract is: form-family id, state layout, truth view interface, input provider interface, `IContinuousSystem` ownership, publish-phase behavior, observable fields, and the way interaction components close environment / process / output data into form input.

Decision:
- Every component assembled in `vehicles[].form.components` must declare a non-empty form family.
- A single vehicle may select exactly one form family.
- Interaction components remain form-specific closures and must continue to declare a non-empty form family.
- P2 uses `contract_6dof` only as a test fixture and extension example; it is not a builtin physical 6DoF model.

## ADR-009: Hybrid Continuous/Discrete Boundary

P2 does not introduce `IHybridSystem`. `update()` remains the discrete entrypoint, and continuous state advancement remains owned by `IContinuousSystem::computeDerivatives()` plus the configured integrator.

Decision:
- Continuous actuator, filter, mass, or mechanism state should live in an `IContinuousSystem` component.
- Discrete sampling, commands, state machines, and mode switching should live in process/output components.
- The two sides communicate through explicit project interfaces.
- A future hybrid API must be designed as a separate, explicit two-entry contract instead of changing `update()` semantics.

## ADR-010: Service And Controlflow Boundary

Services are infrastructure. They may reduce repeated setup for stable framework-owned capabilities, but they must not absorb mission physics, software-process logic, output capability logic, or interaction closure logic.

Decision:
- Controlflow should be modeled as a project `vehicle.process` component, optionally using `gnc::libraries::StateMachine`.
- P2 does not add a builtin controlflow service.
- P2 does not open project service package registration.
- Guidance/control outputs must still enter equations through a form-specific interaction component.

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

## ADR-006B: 配置解析与预处理解耦
配置输入分为 parser facade 和预处理层。`ConfigManager` 不直接依赖内建
JSON parser；它只接收 `IConfigParser` 输出的 `ConfigNode`，再执行
`$include` 展开和 deep-merge。

决定：
- `loadFromFile()` 支持相对 include、`repo://`、`project://`、`user-data://`。
- 多 include 按顺序 deep-merge，本地字段覆盖 include，数组整体替换。
- `loadFromString()` 不支持 filesystem include。
- 第三方 JSON/YAML parser 接入时应替换 parser 实现，不改变 mission assembly。

## ADR-007: 多飞行器 snapshot 后置

本阶段只文档化跨 vehicle 读取应理解为发布态读取，不实现 world snapshot API，也不禁止现有 direct lookup。snapshot API 和 direct lookup 限制作为第二阶段单独设计。
