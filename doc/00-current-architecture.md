# 当前架构总览

GNCZMKN 当前是显式装配的固定步长仿真框架。mission 使用顶层 `vehicles[]` 描述广义飞行器实体；每个实体拥有自己的 `form/services/common/input/process/output/interaction` 块，并注册到 `<id>.<name>` 作用域。

旧式 `entities[]`、根级 `components/services`、根级 `form/vehicle/interaction` 会被运行时拒绝。

## 职责边界

| 概念 | 职责 |
| --- | --- |
| `Form` | 连续状态、导数方程、truth view、form input 和 form-local math |
| `Environment` | 地球、大气、重力等只读环境查询 |
| `Vehicle` | 飞行器侧资产、传感器、制导控制、气动、质量和推进能力 |
| `Interaction` | form-specific closure，把 truth、环境、命令和 output 能力组合成 form input |

`vehicle.common` 是非调度的资产/profile 层。运行时物理能力放在 `vehicle.output`，不要放回 `common`。

## 运行时循环

固定步长循环以周期开始发布态为切断点：

```text
周期 k 开始，时间 t_k
1. publish / refresh：刷新当前连续状态对应的 truth/view/observer
2. before_step(step, t_k, dt)
3. discrete update(t_k)：environment/input/process/output/interaction/form 中的离散组件
4. record t_k：写出发布态和本周期离散输出
5. stop check t_k：检查同一个发布态
6. synchronized independent integration：所有连续系统从 t_k 推进到 t_{k+1}
7. 进入下一周期
```

CSV 每一行对应 `time = t_k`。form/dynamics/truth view 字段表示周期开始发布态 `x_k` 及其真实派生量；input/process/guidance/output 字段表示 `update(t_k)` 后组件当前暴露的本周期离散输出。`t0` 行包含初始状态 `x_0`，以及基于 `x_0` 计算出的第一周期离散输出。框架不保证所有 observable 在物理意义上无延迟；延迟语义由组件模型自身定义。如果 `t0` 已满足停止条件，框架仍会先 publish、update、record，再终止，不进入 integrate。

`form.local_spherical_3dof.flight_state_view` 是 form/truth 层真实飞行状态视图，在 publish 阶段刷新，供 input、sensor、navigation、guidance 等读取。机上导航或估计飞行状态应作为 `vehicle.process` 组件建模，例如 `navigation_flight_state` 或 `estimated_flight_state`，不要混入 form truth view。

publish 顺序不是原始 JSON 顺序。默认调度器先发布 state owner，再发布 view / observer，最后发布普通组件；可选 `priority` 只在同一 publish phase 或 runtime stage 内排序。
## 连续积分近似

当前不是全局联合 ODE。框架采用同步发布点的独立连续积分：

- 每个 `IContinuousSystem` 独立计算本步 next state。
- 本步导数读取其他系统的 `t_k` 发布态。
- 所有连续系统在本步末统一提交到 `t_{k+1}`。

这消除了连续组件注册顺序导致的“部分新、部分旧”状态，但仍不是多体强耦合积分器。未来如需强耦合，应新增 coupled group 或 state assembler，而不是改变当前 mission schema。

## 多飞行器规则

本阶段多飞行器默认不是连续强耦合系统。跨 vehicle 读取应理解为读取对方已发布的周期边界状态；常规目标机、拦截器、观测器场景应优先通过 observer/service 表达。world snapshot API 后续单独设计，本阶段不限制现有 direct lookup。
