# 06 如何实现自定义扩展

## 6.1 自定义扩展的最小原则

在这个框架里，自定义扩展的最佳做法不是修改核心代码，而是：

1. 选择一个合适的接口
2. 写一个继承 `ComponentBase` 的组件类
3. 放到 `user/components/` 目录下
4. 用注册宏登记类型
5. 在 mission JSON 里实例化它

只要沿着这条路径走，框架的自动装配优势就能保留下来。

## 6.2 应该实现哪个接口

先按职责选接口，而不是先想类名。

### 如果你在做制导

根据任务类型选：

- `IGuidance6DOF` 或别名 `IGuidance`
- `IGuidance3DOF`
- `IGuidanceOverload`

仓库中的 `ConstantGuidance` 和 `CavhProgrammedAoA` 分别对应 6DOF 和 3DOF 两种思路。

### 如果你在做导航

实现：

- `INavigation`

导航组件通常依赖：

- 传感器接口
- 动力学接口
- 真值状态接口

### 如果你在做动力学

实现：

- `IDynamicsModel`

通常还建议一并实现：

- `IPositionProvider`
- `IVelocityProvider`
- `IObservable`

这样其他组件更容易直接复用你的状态。

### 如果你在做环境、气动、质量、传感器、控制

优先看下列接口：

- `IAtmosphereModel`
- `IGravityModel`
- `IEarthModel`
- `IAeroCoefficients`
- `IMassProperty`
- `IImuSensor`
- `IGpsSensor`
- `IController`

## 6.3 一个标准组件应当包含什么

一个设计良好的组件通常包括以下部分：

| 部分 | 建议内容 |
| --- | --- |
| 构造函数 | 设置组件名、默认执行频率、初始化内部状态 |
| `configure()` | 读取 mission 中的参数并设置默认值 |
| `injectDependencies()` | 从 `ScopedRegistry` 获取依赖组件 |
| `injectServices()` | 获取服务对象 |
| `initialize()` | 做合法性检查、准备缓存 |
| `update()` 或 `computeDerivatives()` | 写核心算法 |
| `getObservableFields()` | 暴露你希望记录/停止判断的关键字段 |

## 6.4 组件应该放在哪

推荐目录：

- `user/components/guidance/`
- `user/components/navigation/`
- `user/components/controller/`

如果你的模型更复杂，也可以继续分子目录。顶层 CMake 会递归扫描所有 `*.hpp` 文件。

## 6.5 为什么只推荐放头文件

当前仓库的自动注册和扫描机制是围绕头文件工作的。现有模板和用户目录约定也明确倾向：

- 一个组件一个 `.hpp`

这样做的好处是：

- 自动包含和注册简单
- 不需要改构建脚本
- 便于模板化复制

如果后续你要引入更复杂的 `.cpp` 组织方式，也可以做，但那已经超出当前仓库默认工作流。

## 6.6 自动注册是如何生效的

每个组件头文件末尾都应使用注册宏，把“类名”和“它实现的接口”登记进工厂。框架随后才能在 mission 中通过 `type` 名找到该类型。

用户需要记住的不是宏细节，而是这条规则：

- mission 中的 `type` 必须和注册名一致

最安全的做法是让注册名直接等于类名。

## 6.7 依赖注入怎么写才稳健

### 同作用域依赖

多数情况下，在 `injectDependencies()` 中通过短名查找即可，例如：

- `guidance`
- `dynamics`
- `imu`
- `nav`

这是因为 `Simulator` 会根据组件全名自动推导作用域，并用 `ScopedRegistry` 注入。

### 跨实体依赖

如果你在多实体场景下需要访问其他实体，使用全名：

- `target.dynamics`
- `env.gravity`

这在 `tests/test_cross_entity_access.cpp` 中有直接验证。

### 必需依赖和可选依赖

如果某个依赖缺失会让组件失去意义，建议额外实现 `IDependencyDeclarer`，显式声明依赖。这样 `SimulationBuilder` 会在构建阶段给出更清晰的错误，而不是等运行后空指针行为。

## 6.8 服务注入怎么理解

除了组件依赖，框架还支持服务依赖。当前唯一已落地的服务是：

- `CoordinateService`

如果 mission 里启用了对应服务，组件可在 `injectServices()` 里获取它。服务机制适合承载：

- 通用坐标转换
- 星历
- 全局时钟
- 统一场景信息

不适合承载具体飞行器算法状态。

## 6.9 如何让组件自动出现在 CSV 里

只实现组件功能还不够。如果你想让用户在 mission 中通过 `outputs.record` 记录你的组件数据，就应该实现：

- `IObservable`

推荐使用：

- `ObservableFieldBuilder`

来组织标量、三维向量、四元数字段。

好的字段命名建议：

- 有物理语义
- 可以直接出现在 CSV 列名中
- 不依赖内部代码上下文也能理解

例如：

- `position.x`
- `mach`
- `alpha_deg`
- `lift_to_drag`

## 6.10 如何为停止条件准备字段

停止条件和 CSV 记录共用同一套 `IObservable` 字段体系。因此，任何可能成为任务终止判据的量，都建议暴露为可观测字段，例如：

- 高度
- 速度
- 能量
- 质量
- 误差量

这样 mission 就能在不改 C++ 的情况下切换终止策略。

## 6.11 如何设计配置项

建议遵守以下原则：

- 键名直接对应物理意义
- 单位写在文档或 `_comment` 中
- 提供合理默认值
- 对旧参数名是否兼容，要明确决定

从现有代码看，兼容旧键名是被允许的，但不宜滥用。新组件最好尽早稳定参数名。

## 6.12 开发一个新组件的推荐工作流

1. 先从 `templates/` 复制最接近的模板
2. 改类名、注册名和接口
3. 只保留真正需要的依赖
4. 在 `configure()` 中定义清晰参数
5. 先让组件在最小任务里单独工作
6. 补 `IObservable` 字段
7. 再把它放进完整任务

这个工作流能显著降低排错难度。

## 6.13 当前模板文件各自适合什么

| 模板 | 用途 |
| --- | --- |
| `templates/guidance_template.hpp` | 新制导组件起点 |
| `templates/navigation_template.hpp` | 新导航组件起点 |
| `templates/controller_template.hpp` | 新控制组件起点 |
| `templates/mission_template.json` | 新任务配置起点 |

这些模板的价值不在于“拿来即用”，而在于把框架期望的扩展姿势固定下来。

## 6.14 当前代码给出的几个重要经验

### 经验一：先做能跑通的最小闭环

`ConstantGuidance` 之所以有价值，不是因为算法高级，而是因为它把“自定义组件 + 自动注册 + 配置装配 + 输出记录”这条链路全部走通了。

### 经验二：把复杂模型拆成多个小组件

CAV-H 示例没有把质量、气动、制导、动力学写成一个类，而是拆成多个接口明确的组件。这种拆法是值得沿用的。

### 经验三：让组件对外暴露“角色”，不要暴露“内部实现”

例如动力学对外暴露 `IDynamicsModel`、`IPositionProvider`、`IVelocityProvider`，而不是要求别人知道它的具体状态存储细节。

## 6.15 常见扩展错误

### 只写了类，没注册

结果是 mission 中永远找不到该组件类型。

### `name` 写得随意

结果是其他组件按约定短名取不到依赖。

### 想记录数据，但没实现 `IObservable`

结果是框架能运行，但 CSV 和停止条件都无法使用你的关键变量。

### 在动力学组件的 `update()` 里推进状态

结果是和积分器路径冲突，或者根本不走积分器。

### 把服务当作业务状态容器

结果是作用域和职责边界会变乱。服务应保持“跨组件共享基础设施”的定位。
