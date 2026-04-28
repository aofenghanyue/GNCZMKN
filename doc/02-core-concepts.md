# 核心概念

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

## Update 契约

`update(dt)` 是离散周期入口：

```text
update 在每个周期最多调用一次。
update 读取 t_k 发布态。
update 产生用于 [t_k, t_{k+1}] 的本周期离散输出。
update 不负责连续状态积分。
```

连续组件若保留 `update()`，不应在里面推进状态；刷新 truth/view/observer 派生量应放在 `publish(time)`。

## Publish 顺序

publish 先按 phase 分组：state owner 先发布，view / observer 随后，普通组件最后。可选 `priority` 只在同一 phase 或 execution stage 内排序；priority 相同保持 mission 顺序。

## 记录和停止条件

固定步长循环在周期开始记录发布态：

```text
publish/refresh t_k -> before_step(t_k) -> update(t_k)
-> record t_k -> stop check t_k
```

CSV 的 `time` 列就是该行状态的物理时间。form/dynamics/truth view 字段来自周期开始发布态 `x_k` 及其真实派生量；input/process/guidance/output 字段来自 `update(t_k)` 后组件当前暴露的本周期离散输出。`t0` 行包含初始状态 `x_0` 和基于 `x_0` 计算出的第一周期离散输出。框架不保证所有 observable 在物理意义上无延迟；延迟语义由组件模型自身定义。停止条件使用同一个发布态，并在 record 之后检查，因此触发停止的状态默认会出现在最后一行 CSV 中。
