# 当前架构总览

GNCZMKN 当前是显式装配的仿真框架。mission 不再描述旧式隐式装配图，也不再使用 `entities[]`。用户通过固定的顶层块声明一次仿真需要哪些状态、环境、飞行器组件、interaction、输出和停止条件。

## 四个顶层概念

| 概念 | 职责 |
| --- | --- |
| `Form` | 拥有被积分状态、状态导数、truth view、form input 和 form-local math |
| `Environment` | 提供地球、大气、重力等环境查询能力 |
| `Vehicle` | 组织飞行器侧的资产、传感器、软件过程和运行时物理效应 |
| `Interaction` | 把 form truth、环境查询、process 命令和 output 能力闭合成 form input |

这个边界的核心原则是：状态推进属于 `Form`，环境查询属于 `Environment`，飞行器侧能力属于 `Vehicle`，跨边界闭合属于 `Interaction`。

## Mission 顶层结构

当前 mission 使用这些顶层块：

```text
simulation
form
environment
vehicle
interaction
outputs
stop_conditions
global_services
```

`global_services` 可省略。旧式 `entities[]`、根级 `components` 和根级 `services` 会被当前 runtime 拒绝。多飞行器任务使用现代 `vehicles[]` 布局，其中每个条目都有自己的 `id`、`form`、`services`、`common/input/process/output` 和 `interaction`；组件注册到 `<id>.<name>`，例如 `chaser.dynamics` 和 `target.dynamics`。

## Vehicle 分层

`vehicle` 内部按用户可见职责分成四类组件：

| 块 | 是否调度 | 职责 |
| --- | --- | --- |
| `vehicle.common` | 否 | 静态资产、profile、参数包、被动初始化数据 |
| `vehicle.input` | 是 | 传感器、测量侧硬件和输入侧运行时包 |
| `vehicle.process` | 是 | 导航、制导、控制、时序逻辑和命令生成 |
| `vehicle.output` | 是 | 气动、质量、推进、分离、构型切换等运行时物理效应 |

不要把运行时气动或质量模型放回 `vehicle.common`。资产文件可以被 output 组件加载，但资产本身不是运行时组件。

## 运行时调度

每个固定步长仿真步按以下顺序执行：

1. `environment`
2. `vehicle.input`
3. `vehicle.process`
4. `vehicle.output`
5. `interaction`
6. `form`
7. 自动记录和停止条件检查

`vehicle.common` 不在调度链中。它只能作为非调度资产/profile 层。

## 注册模型

组件注册是显式的：

- framework builtins 通过 `gnc/bootstrap/register_builtin_packages.hpp` 注册。
- service builtins 通过各自 service package bootstrap 注册。
- 项目组件通过 CMake 生成的 active project 注册链纳入编译。

`GNC_REGISTER_COMPONENT_TYPE` 只应出现在构建系统会扫描到的项目组件头文件中，并且必须声明 package role、execution stage 和 form family。没有隐藏的静态注册兜底路径。

## 当前公开基线

当前推荐从 `user/example_02_atmospheric_3dof/config/mission.json` 理解主流程：

- local-spherical 3DoF form 推进状态。
- environment 提供地球、大气和重力。
- `vehicle.process.programmed_aoa` 生成气动制导命令。
- `mass.constant` 和 `aero.table2d` 作为 `vehicle.output` 运行时能力。
- `interaction.local_spherical_3dof.aero_propulsive` 计算 form input。
- `outputs.record` 记录 observable 字段。
