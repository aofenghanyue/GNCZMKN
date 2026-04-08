# 01 设计理念与总体架构

## 1.1 框架的定位

这个框架的设计重点，不是把某一种具体导弹/飞行器模型写死，而是提供一套能持续承载不同仿真任务的通用底座。代码中的核心机制主要围绕五件事展开：

1. 组件生命周期管理
2. 组件注册与按类型实例化
3. 作用域内依赖注入
4. 仿真时钟与频率调度
5. 配置驱动的数据记录

因此，用户真正需要理解的不是“某个类做了什么”，而是“框架希望你按什么方式组织模型”。

## 1.2 架构分层

当前仓库大体可以分成下面几层：

| 层次 | 目录 | 作用 |
| --- | --- | --- |
| 基础数学与通用类型层 | `framework/include/gnc/common` | 数学类型、Eigen 扩展、旋转/变换、数值计算、日志等基础能力 |
| 抽象接口层 | `framework/include/gnc/interfaces` | 定义环境、动力学、导航、制导、控制、传感器、输出等接口契约 |
| 框架核心层 | `framework/include/gnc/core` | 组件基类、注册表、工厂、配置解析、仿真器、构建器、自动数据记录等 |
| 起步件层 | `framework/include/gnc/components` | 当前仓库随附的 starter components，服务于冷启动、最小闭环和示例 |
| 服务与复用库层 | `framework/include/gnc/services`、`framework/include/gnc/libraries` | 坐标服务、PID、滤波器、状态空间、控制离散化等可复用能力 |
| 用户扩展层 | `user/components`、`user/config` | 用户自定义组件和任务配置 |
| 示例与验证层 | `examples`、`tests` | 示例模型、用法演示与基础验证 |

这里有一个需要特别强调的现实边界：

- `framework/include/gnc/components` 目前在物理目录上位于 `framework/` 下
- 但在逻辑定位上，应把它看成“随仓库附带的起步件集合”，而不是 framework core 本身

## 1.3 四个核心设计思想

### 配置驱动，而不是入口硬编码

入口程序 `src/runner.cpp` 的职责非常轻。它只做三件事：

- 加载配置文件
- 触发内建组件和用户组件注册
- 通过 `SimulationBuilder` 构建仿真器并运行

因此，大多数仿真任务的差异不需要通过修改 `main()` 表达，而是通过：

- 在 `user/components/` 中添加新组件
- 在 mission JSON 中替换 `type`、调整 `name` 和 `config`

### 接口优先，而不是类名耦合

组件之间不是按具体类耦合，而是按接口耦合。典型例子：

- 导航组件依赖 `IDynamicsModel`、`IPositionProvider`、`IVelocityProvider`
- 3DOF 动力学依赖 `IAtmosphereModel`、`IGravityModel`、`IAeroCoefficients`、`IMassProperty`、`IGuidance3DOF`
- 传感器依赖动力学或状态提供接口

这样做的结果是：你可以保留同一个任务配置结构，只替换某一层实现，而不必连带修改全局代码。

### 观测优先，而不是手写日志

数据输出不是靠每个组件自己写文件，而是通过 `IObservable` 暴露“可记录字段”，再由 `AutoDataLogger` 根据 `outputs.record` 自动发现并记录。

这意味着：

- 组件作者只负责声明哪些量值得被观察
- 任务作者只负责在 JSON 中说明要记录哪些组件/字段
- 输出格式、列名和记录时机由框架统一处理

这是当前框架里最值得保留和继续扩展的一条设计主线。

### 为多实体场景预留作用域

`SimulationBuilder` 已经支持：

- 单飞行器配置
- 环境实体 `environment`
- 多飞行器配置 `vehicles`

组件注册名在多实体模式下会自动带作用域前缀，例如：

- `env.gravity`
- `chaser.nav`
- `target.dynamics`

`ScopedRegistry` 负责在同作用域中解析短名，也允许用全名访问其他实体组件。这说明框架从底层就考虑了多飞行器/环境共享服务/跨实体观测这类扩展方向。

## 1.4 一次仿真是如何运行的

从用户视角看，一次仿真执行流程可以概括为：

1. CMake 收集 `user/components/**/*.hpp`
2. 生成 `build/generated/user_components_register.hpp`
3. 程序启动后，静态注册宏把所有组件类型登记到 `ComponentFactory`
4. `SimulationBuilder` 读取 JSON
5. 根据 `type` 创建组件实例
6. 用 `name` 将组件注册到 `ComponentRegistry`
7. 调用 `configure()` 读取组件参数
8. 调用 `injectServices()`、`injectDependencies()` 注入服务和其他组件
9. `Simulator` 根据仿真步长和组件频率执行主循环
10. 对 `IDynamicsModel` 组件使用积分器推进状态
11. 对连续动力学组件执行积分后的 `update(dt)` 后处理钩子，对普通组件调用 `update(dt)`
12. `AutoDataLogger` 记录当前步的数据
13. 满足停止条件或达到结束时间后收尾，输出 `summary.txt`

## 1.5 生命周期与调度规则

每个组件都继承 `ComponentBase`。用户最需要关心的生命周期钩子有：

| 方法 | 什么时候调用 | 你通常在这里做什么 |
| --- | --- | --- |
| 构造函数 | 创建实例时 | 设置组件名、默认频率、状态布局等 |
| `configure()` | 读取 mission 配置时 | 读取 JSON 参数、初始化默认值 |
| `injectDependencies()` | 构建后、初始化前的依赖绑定阶段 | 获取其他组件接口指针；应保持为无副作用绑定逻辑 |
| `injectServices()` | 构建阶段 | 获取服务对象，例如坐标服务 |
| `initialize()` | 开始仿真前 | 做一致性检查、准备缓存 |
| `update()` 或 `computeDerivatives()` | 每个执行周期 | 写核心算法 |
| `finalize()` | 仿真结束时 | 收尾工作 |

调度上有两个重要规则：

- 如果组件实现了 `IDynamicsModel`，仿真器不会依赖它的 `update()` 来推进状态，而是通过积分器调用 `computeDerivatives()`
- 连续动力学组件的 `update()` 适合放积分后的后处理逻辑，而不应再次做积分
- 组件频率通过 `setExecutionFrequency()` 指定，仿真器会自动换算为步进间隔

因此，动力学组件和普通组件的写法是不同的。不要把“状态推进逻辑”塞回 `update()`，否则会与框架的积分路径脱节。

## 1.6 依赖声明的两档策略

当前框架对依赖装配采用两档策略，而不是要求所有组件一律实现 `IDependencyDeclarer`：

- 轻量组件：直接在 `injectDependencies()` 中使用 `bind(...)` / `bindIfPresent(...)`
- 装配型组件：在此基础上继续实现 `IDependencyDeclarer`

这里的区别不在于“有没有依赖”，而在于“显式声明是否真的能提供额外价值”。

适合只用 `bind(...)` 预检查的情况：

- 依赖数量很少
- 都是同作用域短名绑定
- `getDependencies()` 只是机械重复 `injectDependencies()` 里的必需绑定

适合继续保留 `IDependencyDeclarer` 的情况：

- 依赖较多，缺失时希望一次性报出整组装配问题
- 依赖描述本身具有明确的任务语义
- 多实体装配中需要更完整的构建期诊断

当前仓库已经按这条边界开始收口：

- `SimpleNavigation`、`TruthState`、`CavhProgrammedAoA` 这类轻量组件回到了 `bind(...)` 预检查
- `Dynamics3DOF_SphericalEarth`、`CavhAerodynamics` 这类装配型组件仍保留显式依赖声明

并且现在 declarer 也不是“只看声明、不看真实绑定”的特殊通道：

- 显式依赖声明先负责提供更完整的装配语义
- 随后的依赖预检查仍会执行真实 `injectDependencies()`

因此 declarer 的定位已经更清楚了：它是对绑定语义和诊断质量的增强，而不是绕开真实注入逻辑的替代路径。

## 1.7 当前代码中体现出的设计取舍

从现有实现可以看出几个很鲜明的工程取舍：

- 选择轻量级自研 JSON 解析器，减少外部依赖
- 选择头文件框架，降低链接复杂度，方便模板化扩展
- 选择运行期按字符串装配组件，提升任务配置灵活性
- 选择按接口而不是按模块强耦合，提升替换性
- 选择“最小可用内建组件 + 强扩展骨架”，避免过早做成庞大单体

如果你后续继续演进这个框架，这些取舍应该被视为主线，而不是临时实现细节。

## 1.8 用户应如何理解这个框架

最合适的理解方式是：

- `core/` 负责“怎么跑”
- `interfaces/` 负责“怎么接”
- `components/` 负责“先给你一些能用的实现”
- `user/components/` 负责“你自己的模型”
- `user/config/` 负责“这次任务怎么装起来”

只要把这五者关系理解清楚，整个框架的设计就不会乱。
> 2026-04-08 补充说明
>
> - `SimulationBuilder::build()` 负责实例化组件、做依赖校验、执行真实 `injectDependencies()` 预检查、构建 stop condition，并初始化数据记录器。
> - `Simulator::initialize()` 发生在真正开跑前：它只会对尚未在构建期标记为“依赖已注入”的组件补做依赖注入，然后调用各组件的 `initialize()`。
> - 因此 `injectDependencies()` 应尽量保持为无副作用的绑定阶段；一次性缓存准备和更重的初始化更适合放在 `initialize()`。
> - 当前仓库已经不再依赖重复 declarer 才能获得整组缺依赖诊断；包括 `Dynamics3DOF_SphericalEarth`、`CavhAerodynamics` 在内，诊断主路径都已经回到真实 `injectDependencies()` 预检查。
> - `IDependencyDeclarer` 仍可使用，但定位已经收紧成“显式依赖契约增强层”，而不是默认必需层。
