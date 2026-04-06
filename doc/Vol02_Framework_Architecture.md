# Volume 2: 框架架构与核心设计模式 (Architecture)

## 1. 设计哲学：去总线化与主动拉取 (Pull Model)
在传统的 ROS 或类似仿真框架中，数据通常通过“发布-订阅（Pub/Sub）”和序列化/反序列化进行传递。这在分布式系统中很好，但在单机高性能 GNC 闭环仿真中，带来了巨大的延迟和内存拷贝开销。

本框架摒弃了消息总线，采用了**接口契约 + 依赖注入 + 主动拉取**的架构。

### 1.1 工作原理
- **注册与依赖声明**：组件 A（如制导）向 `ComponentRegistry` 注册自己实现了 `IGuidance3DOF` 接口。组件 B（如气动）声明自己依赖 `IGuidance3DOF` 接口。
- **依赖注入**：在仿真初始化阶段（`initialize` 前），`Simulator` 通过 `ScopedRegistry` 将组件 A 的指针注入给组件 B。
- **零拷贝拉取**：在主循环的 `update(dt)` 中，组件 B 直接调用 `guidance_->getFlightCommand()`。**这是一个虚函数调用，直接返回内存引用，没有任何数据拷贝。**

## 2. 五层架构设计

框架严格按照领域驱动设计（DDD）划分为五层：

1. **应用层 (Application)**:
   - 包含 `SimulationBuilder`、`Simulator` 以及具体业务场景。
   - 负责解析 JSON、实例化组件、组织生命周期。
2. **服务层 (Service)**:
   - 跨组件的全局设施，例如 `CoordinateService`（坐标树查找）和 `AutoDataLogger`（自动落盘）。
   - 通过 `ServiceContext` 按需获取，不污染核心业务逻辑。
3. **接口层 (Interface)**:
   - 极其关键的“契约”层。包含 `IDynamicsModel`、`IGuidance`、`IAtmosphereModel` 等。
   - 组件之间的通信完全基于接口，实现了完美的解耦。
4. **算法库 (Algorithm Library)**:
   - 提供复用的 GNC 算法模块，如 `PidController`、`ButterworthFilter`、`StateSpaceModel`。
5. **数学库 (Math Library)**:
   - 底层基石。包含 `Quaternion`、`Rotation`、`rk45_step`（自适应积分）、`LookupTable3D` 等纯数学工具。

## 3. 组件的生命周期与阶段管理

每个组件都继承自 `ComponentBase`，其生命周期由 `Simulator` 内部的 `ExecutionPhaseManager` 严格管理，防止时序错误。

| 阶段 (Phase) | 触发的方法 | 作用 |
|-------------|-----------|------|
| `NotStarted`| `configure(const ConfigNode&)` | 从 JSON 中读取初始参数 |
| `Initializing`| `injectDependencies(ScopedRegistry&)` | 建立组件间的指针连接 |
| `Initializing`| `initialize()` | 首次运行前的初始化（分配内存、预计算） |
| `Running`   | `shouldExecute(step)` -> `update(dt)` | 仿真主循环，根据组件频率调度 |
| `Finalizing`| `finalize()` | 仿真结束后的清理工作 |

### 频率调度机制
组件不是每一步都必须执行。您可以在构造函数中调用 `setExecutionFrequency(50.0)`。`Simulator` 会根据全局步长 `dt`，计算出 `step_interval`。例如 `dt=0.01` (100Hz)，组件频率 50Hz，则 `shouldExecute` 每隔 1 步返回 true。

## 4. 多飞行器隔离 (Scoped View)
当仿真场景包含多个飞行器（如导弹拦截靶机）时，同名组件会发生冲突（如都有 `dynamics` 和 `guidance`）。
框架通过 `ScopedRegistry` 解决：
- 注册表中，追踪器的组件名为 `chaser.dynamics`，目标的名为 `target.dynamics`。
- 当追踪器的制导组件请求 `dynamics` 时，`ScopedRegistry` 会自动加上前缀 `chaser.` 去查询。
- 全局环境组件（如 `env.atmosphere`）没有前缀限制，所有飞行器共享。