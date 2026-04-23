# 核心概念

GNCZMKN 的文档和代码都围绕几个稳定问题组织：谁拥有状态，谁提供环境能力，谁代表飞行器侧能力，谁把这些能力闭合成状态输入。

## Form

`Form` 是状态和积分方程所在的位置。它负责：

- 定义状态变量和状态导数。
- 暴露 truth view，例如 Cartesian 或 local-spherical truth。
- 接收 form input。
- 提供 form-local math 和 observable 字段。

内置示例：

- `form.cartesian_3dof.point_mass`
- `form.local_spherical_3dof.point_mass`
- `form.local_spherical_3dof.flight_state_view`

Form 组件在 registry 中使用 `vehicle.` 前缀。例如名为 `dynamics` 的 form 组件全名是 `vehicle.dynamics`。

## Environment

`Environment` 提供世界侧查询能力，例如：

- 地球模型: `IEarth`
- 大气模型: `IAtmosphere`
- 重力模型: `IGravity`

环境组件使用 `env.` 前缀。例如名为 `atmosphere` 的组件全名是 `env.atmosphere`。

## Vehicle

`Vehicle` 组织飞行器侧能力。它不是一个单一大类，而是四个用户可见分区：

| 分区 | 职责 |
| --- | --- |
| `common` | 静态资产、profile、参数包、被动初始化数据 |
| `input` | 传感器和测量侧硬件 |
| `process` | 导航、制导、控制和命令生成 |
| `output` | 气动、质量、推进、分离和构型切换等运行时效应 |

`common` 不参与每步调度。`input/process/output` 分别映射到运行时 stage。

## Interaction

`Interaction` 是 form-aware closure 层。它读取：

- form truth
- environment 查询
- `vehicle.process` 命令
- `vehicle.output` 运行时能力

然后写入 selected form input。

这意味着 interaction 不应该拥有气动表、质量定义或推进模型。它消费这些 output 组件暴露的接口。

## 资产、Loader 与运行时组件

这三类对象不要混淆：

| 类型 | 例子 | 是否是组件 |
| --- | --- | --- |
| 资产文件 | `framework/data/vehicles/cavh/output/aero_table2d.json` | 否 |
| loader/parser | 读取 JSON 或 CSV 的工具代码 | 否 |
| 运行时组件 | `aero.table2d`、`mass.constant` | 是 |

资产文件可以被组件在 `configure()` 或 `initialize()` 中加载，但资产文件本身不进入调度，也不暴露接口。

## 执行顺序

每个仿真步按固定顺序执行：

1. `environment`
2. `vehicle.input`
3. `vehicle.process`
4. `vehicle.output`
5. `interaction`
6. `form`

自动记录和停止条件检查发生在 step 层。当前运行循环围绕固定步长 `simulation.dt` 组织。

## 组件名和 Type Id

mission 中每个组件都有两个名字：

| 名称 | 来源 | 用途 |
| --- | --- | --- |
| type id | `type` 字段 | 从 factory 创建组件，例如 `aero.table2d` |
| local name | `name` 字段 | 形成 registry 全名，例如 `vehicle.aero` |

常见全名规则：

| 放置位置 | local name | 全名 |
| --- | --- | --- |
| `environment.components` | `earth` | `env.earth` |
| `form.components` | `dynamics` | `vehicle.dynamics` |
| `vehicle.process` | `guidance` | `vehicle.guidance` |
| `vehicle.output` | `aero` | `vehicle.aero` |
| `interaction.components` | `interaction` | `vehicle.interaction` |

同一 vehicle 作用域内的依赖通常可以写 local name；跨环境引用应写完整名，例如 `env.earth`。

## Role、Stage 与 Form Family

每个注册组件可以声明：

- `ComponentPackageRole`: 组件应该放在哪个 mission 块。
- `ExecutionStage`: 组件在哪个 stage 执行，或 `None`。
- `form_family`: 组件适配的 form family，或空字符串表示 form-neutral。

assembly 会检查 mission 放置位置与注册元数据是否一致。例如把 `aero.table2d` 放进 `vehicle.process` 会失败，因为它注册为 `vehicle_output`。
