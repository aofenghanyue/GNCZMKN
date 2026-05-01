# 核心概念

这篇文档先建立心智模型：GNCZMKN 不是把所有物理和算法写进一个全局仿真对象，而是用 mission 显式装配一组带边界的组件。理解这四层后，再读配置和源码会容易很多。

| 层 | 读者要理解的事 |
| --- | --- |
| Mission | JSON 只描述“有哪些对象、组件和服务”，不写 C++ 控制流 |
| Component | 组件通过接口暴露能力，通过 role/stage/form family 约束放置位置 |
| Service | 服务提供稳定基础设施能力，例如坐标树；不承载任务流程或物理闭合 |
| Runtime loop | 固定步长循环在发布态、离散 update、记录、停止条件和连续积分之间切换 |

一个典型数据闭环如下：

```text
Form state -> truth view -> process/output/environment queries
-> interaction closure -> form input -> computeDerivatives()
-> next form state -> publish -> observable record
```

其中 `truth` 是 form 发布的真实状态视图；`form input` 是 interaction 给 form 的输入；`process command` 是导航/制导/控制等离散算法输出；`output capability` 是气动、质量、推进等物理能力；`observable` 是组件愿意稳定记录到 CSV 的字段。

## Form

`Form` 是连续状态和导数方程所在的位置。它负责定义状态变量、计算导数、发布 truth view，并消费 form input。内置示例包括：

- `form.cartesian_3dof.point_mass`
- `form.local_spherical_3dof.point_mass`
- `form.local_spherical_3dof.flight_state_view`

连续 form 在周期开始的 `publish()` 中刷新 truth/view；连续状态推进由积分器调用 `computeDerivatives()` 完成。`form.local_spherical_3dof.flight_state_view` 是 truth/form 层真实飞行状态视图，也在 publish 阶段刷新。机上导航或估计飞行状态应放在 `vehicle.process` 组件中，而不是混入 form truth view。

## Environment

`Environment` 提供世界侧查询能力，例如 `IEarth`、`IAtmosphere`、`IGravity`。环境查询应尽量是只读计算，不在查询函数中推进离散状态。

## Vehicle

`Vehicle` 按职责拆成四个用户可见分区：

| 分区 | 职责 |
| --- | --- |
| `common` | 静态资产、profile、参数包；不参与 runtime stage |
| `input` | 传感器和测量侧硬件 |
| `process` | 导航、制导、控制、时序逻辑和命令生成 |
| `output` | 气动、质量、推进、分离和构型切换等运行时能力 |

气动表、质量定义、推进模型属于 `vehicle.output`，不属于 `interaction`。

## Interaction

`Interaction` 是 form-aware closure。它读取 form truth、环境查询、`vehicle.process` 命令和 `vehicle.output` 能力，然后生成 selected form input。

普通用户应优先配置内置 interaction，例如 direct acceleration 或 aero-propulsive closure。自定义 interaction 必须声明非空 form family，因为它直接生成某个 form 的 input。

## 数据角色

| 名称 | 产生者 | 消费者 | 说明 |
| --- | --- | --- | --- |
| Truth view | `Form` 或 form view | process、interaction、service spec、记录器 | 周期开始发布态对应的真实状态和派生量 |
| Form input | `Interaction` | `Form::computeDerivatives()` | 当前步连续方程使用的加速度、力或其它 form-specific 输入 |
| Process command | `vehicle.process` | interaction 或其它 process/output | 导航、制导、控制和时序逻辑的离散输出 |
| Output capability | `vehicle.output` | interaction、process、记录器 | 气动、质量、力、推进等能力，可能是只读模型或连续状态模型 |
| Observable field | 实现 `IObservable` 的组件 | `AutoDataLogger`、termination | 稳定输出字段，不等于全部内部状态 |
| Debug snapshot | 任意 `ComponentBase` 子类 | debug snapshot CSV | 调试量，字段契约可以比 observable 更松 |

读代码时可以用这个问题判断边界：这个值是连续状态的一部分、离散命令、物理能力，还是只是输出记录？答案通常决定它应该放在哪个分区。

## 名称和作用域

mission 中的组件 `name` 会被装配器加上作用域前缀：

| 位置 | 运行时完整名 |
| --- | --- |
| `environment.components[].name = "earth"` | `env.earth` |
| `vehicles[].id = "vehicle"` 且组件名 `"dynamics"` | `vehicle.dynamics` |
| 顶层 `termination.name = "termination"` | `mission.termination` |
| 顶层 `summary.name = "summary"` | `mission.summary` |

依赖查找通常在当前 vehicle 作用域内解析短名。例如 `vehicle.dynamics` 默认查找名为 `interaction` 的 form input provider，实际会绑定到 `vehicle.interaction`。跨作用域依赖应使用完整名，例如 `env.atmosphere`。

vehicle id 必须匹配 `[A-Za-z_][A-Za-z0-9_-]*`，并且不能使用保留 id `env`。

## Update 契约

`update(dt)` 是离散周期入口：

```text
update 在每个周期最多调用一次。
update 读取 t_k 发布态。
update 产生用于 [t_k, t_{k+1}] 的本周期离散输出。
update 不负责连续状态积分。
```

连续组件若保留 `update()`，不应在里面推进状态；刷新 truth/view/observer 派生量应放在 `publish(time)`。

连续状态和离散状态要分开建模：

- 连续状态实现 `IContinuousSystem`，由积分器调用 `computeDerivatives()` 并统一提交 `setState()`。
- 离散采样、命令、状态机和模式逻辑放在 `vehicle.process` 或 `vehicle.output` 的 `update()`。
- 二者通过显式接口连接，再由 `interaction` 转成 form input。

## Publish 顺序

publish 先按 phase 分组：state owner 先发布，view / observer 随后，普通组件最后。可选 `priority` 只在同一 phase 或 execution stage 内排序；priority 相同保持 mission 顺序。

离散 update 的 stage 顺序固定为：

```text
environment -> vehicle.input -> vehicle.process -> vehicle.output
-> interaction -> form -> termination -> summary
```

`vehicle.common` 不进入 runtime stage，适合放静态资产和 profile。`rate_hz` 只改变组件是否在某个 step 执行，不改变 stage 顺序。

## 记录和停止条件

固定步长循环在周期开始记录发布态：

```text
publish/refresh t_k -> before_step(t_k) -> update(t_k)
-> record t_k -> stop check t_k
```

CSV 的 `time` 列就是该行状态的物理时间。form/dynamics/truth view 字段来自周期开始发布态 `x_k` 及其真实派生量；input/process/guidance/output 字段来自 `update(t_k)` 后组件当前暴露的本周期离散输出。`t0` 行包含初始状态 `x_0` 和基于 `x_0` 计算出的第一周期离散输出。框架不保证所有 observable 在物理意义上无延迟；延迟语义由组件模型自身定义。停止条件使用同一个发布态，并在 record 之后检查，因此触发停止的状态默认会出现在最后一行 CSV 中。

## 常见误区

| 误区 | 正确理解 |
| --- | --- |
| `interaction` 可以放气动表或质量定义 | 气动、质量、推进等运行时能力属于 `vehicle.output`；interaction 只做闭合 |
| `FlightStateView` 是机上导航估计 | 它是 form/truth 层真实飞行状态视图；导航估计应建模为 `vehicle.process` |
| `update()` 可以顺手积分连续状态 | 连续推进由 `IContinuousSystem::computeDerivatives()` 和积分器完成 |
| `priority` 可以任意改变所有组件顺序 | priority 只在同一 runtime stage 或 publish phase 内排序 |
| 多个连续系统就是联合 ODE | 当前是同步发布点下独立积分，统一提交 next state，不是联合状态向量 |
