# 05 框架内建能力

## 5.1 内建能力分成两类

第一类是“框架基础设施”，包括：

- 仿真器
- 积分器
- 配置解析
- 自动数据记录
- 停止条件
- 组件注册与依赖注入

第二类更准确地说是“仓库随附的 starter components”，包括：

- 地球/重力/大气模型
- 简化动力学
- 3DOF 球形地球动力学
- IMU
- 简单导航
- 真值状态

用户在搭建任务前，应该先知道哪些能力已经内建，哪些还只是接口骨架。

这里要避免一个常见误解：

- 这些 starter components 当前放在 `framework/include/gnc/components` 下
- 但它们的逻辑定位不是 framework core
- 更适合把它们看成“仓库附带的起步件与联调件”

## 5.2 当前可直接在任务文件里使用的组件

### 环境类组件

| 类型名 | 实现接口 | 主要用途 | 备注 |
| --- | --- | --- | --- |
| `Wgs84Earth` | `IEarthModel` | 提供地球半径、自转角速度、简化大地坐标转换 | 当前经纬高/ECEF 转换是简化实现 |
| `SphericalGravity` | `IGravityModel` | 提供球形重力模型 | 适合基础轨迹和再入任务 |
| `StandardAtmosphere` | `IAtmosphereModel` | 提供 1976 标准大气 | 覆盖 0 到 86 km 分层大气 |

### 动力学类组件

| 类型名 | 实现接口 | 主要用途 | 适用场景 |
| --- | --- | --- | --- |
| `SimpleDynamics` | `IDynamicsModel`、`IPositionProvider`、`IVelocityProvider` | 最小化点质点/简化 6DOF 风格动力学 | 教学、联调、框架验证 |
| `Dynamics3DOF_SphericalEarth` | `IDynamicsModel`、`IPositionProvider`、`IVelocityProvider` | 球形地球 3DOF 轨迹模型 | 大气飞行、再入、滑翔类任务 |

### 传感器与状态类组件

| 类型名 | 实现接口 | 主要用途 | 备注 |
| --- | --- | --- | --- |
| `IdealImu` | `IImuSensor` | 从动力学读取状态生成 IMU 数据骨架 | 当前主要输出角速度，线加速度未完整建模 |
| `SimpleNavigation` | `INavigation` | 从动力学/状态提供接口读取导航解 | 更适合作为导航占位件或真值导航 |
| `TruthState` | `IPositionProvider`、`IVelocityProvider` | 输出真值位置速度 | 可作为后续传感器误差模型基础 |

## 5.3 内建组件之间是如何配合的

### 最小链路

最小链路通常是：

- `guidance`
- `dynamics`
- `imu`
- `nav`

其中：

- 制导给动力学输入
- 动力学推进状态
- IMU 和导航从动力学读状态

### 3DOF 工程链路

工程一点的链路是：

- `atmosphere`
- `gravity`
- `mass`
- `aero`
- `guidance`
- `dynamics`

这里动力学成为真正的“汇聚点”，把其他模型的输出统一吸收。

## 5.4 内建组件的配置特征

从当前实现能看出一些配置习惯：

- 频率类参数常使用 `frequency_hz`
- 为兼容旧写法，`SimpleNavigation` 和 `IdealImu` 也接受 `frequency`
- 简单动力学使用 `initial_position` 和 `initial_velocity`
- 3DOF 动力学使用 `initial_state` 对象组织状态初值

这说明框架允许不同复杂度组件采用不同配置风格，但建议新组件尽量保持参数命名清晰一致。

## 5.5 starter components 的依赖装配风格

仓库当前的 starter components 已经不再统一强制实现 `IDependencyDeclarer`，而是按复杂度分成两类：

- 轻量起步件，例如 `SimpleNavigation`、`TruthState`
  直接依赖 `bind(...)` / `bindIfPresent(...)` 和构建期预检查
- 装配型起步件，例如 `Dynamics3DOF_SphericalEarth`
  仍然保留显式依赖声明，用来一次性暴露多项缺失依赖

这条边界的意义是：

- 不让简单组件因为样板声明变重
- 也不牺牲复杂装配组件的诊断质量

## 5.6 内建基础设施能力

### 积分器

当前内建积分器有：

- `rk4`
- `euler`

仿真器会自动把它们应用到实现了 `IDynamicsModel` 的组件上。

### 自动数据记录

当前推荐的数据记录机制是：

- 组件实现 `IObservable`
- 任务通过 `outputs.record` 指定记录规则

它会自动：

- 发现字段
- 生成列名
- 写 CSV
- 在输出目录中生成摘要文件

另外，框架现在还支持 `debug_snapshots`：

- 组件可在 `update()` 中通过 `snapDebug()` 写入当前步调试量
- 框架会把这些量写到单独的长表 CSV
- 这条通道适合中间量调试，不替代正式 `IObservable` 接口

旧的 `CsvDataLogger` 仍保留，但定位已经是兼容旧用法，不是推荐主路径。

### 停止条件

当前停止条件是基于“组件可观测字段阈值”的，这与自动数据记录采用了同一套观测接口，是很统一的设计。

### 组件注册

当前组件注册由两层完成：

- 静态注册宏把类型登记到 `ComponentFactory`
- `SimulationBuilder` 按任务文件实例化并注册到 `ComponentRegistry`

## 5.7 服务层和复用库层

### 坐标服务

`CoordinateService` 是当前唯一实装到服务系统中的服务。它提供：

- 坐标系树形拓扑
- 父子旋转注册
- 按 LCA 搜索路径并组合旋转
- 节点级缓存

当前示例没有大规模依赖它，但它明显是未来多坐标系统任务的基础设施。

### 数学与控制复用库

框架还提供了一大批不会自动参与仿真主循环、但非常适合自定义组件复用的能力：

- PID 控制器
- 状态机
- 连续/离散状态空间模型
- 传递函数到状态空间转换
- 坐标旋转公式
- 常见滤波器
- 数值积分、插值、优化、求根、统计等数学工具

这些库的意义不是“框架帮你自动接好”，而是“你写自己的算法组件时不必从零造轮子”。

## 5.8 哪些接口目前只有骨架，没有内建实现

从接口层可以看到，框架已经为很多能力预留了扩展点，但仓库当前并未提供完整内建组件，例如：

- `IController`
- `IGpsSensor`
- `IActuator`
- `IEngine`
- `IDisturbance`
- `IGuidanceOverload`

这说明框架的方向是对的，但很多任务仍需要用户根据具体项目去补实现。

## 5.9 对用户的直接建议

如果你刚接触这个框架，最实用的策略是：

1. 内建环境和动力学能用就先用
2. 先替换算法组件，不要一上来全替换
3. 需要真实工程模型时，再逐步补质量、气动、传感器和控制层

这样能最大化复用当前框架已经稳定的部分。
> 2026-04-08 补充说明
>
> - starter components 当前默认依赖真实 `injectDependencies()` 预检查，而不是默认依赖 `IDependencyDeclarer`。
> - 这意味着即使是 `Dynamics3DOF_SphericalEarth` 这类多依赖 starter 组件，也已经不需要靠重复 declarer 才能获得整组缺依赖诊断。
> - stop condition 现在优先复用 `IObservable` 字段，但如果目标组件实现了 `IDynamicsModel`，也可以直接读取动力学状态名。
> - 如果同一个量同时出现在 `IObservable` 和 `snapDebug()` 中，它会同时进入主 CSV 和调试 CSV；这是有意保留两条输出通道，不是冲突。
