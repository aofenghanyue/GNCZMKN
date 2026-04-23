# 设计决策

这篇文档记录当前仍然有效的架构取舍。历史迁移材料可以保留在
`temp/seeByAgent/` 或版本历史中，但不能作为当前运行时、用户文档或测试兼容目标的正式依据。

## 1. 任务配置采用显式架构块

当前 mission 顶层结构固定围绕下列块组织：

- `simulation`
- `form`
- `environment`
- `vehicle`
- `interaction`
- `outputs`
- `stop_conditions`
- `global_services`

这个结构直接暴露当前运行时的职责边界：`form` 持有被积分状态，
`environment` 提供环境能力，`vehicle` 组织飞行器侧组件，
`interaction` 负责把环境、过程命令和输出效应闭合成 form input。

当前正式基线是：

- 不支持旧式 `entities[]`
- 不支持旧式根级 `components` / `services` / `vehicles`
- 不为了兼容历史示例恢复旧 schema 分支

## 2. `vehicle` 分层表达运行时职责

`vehicle` 内部使用四个用户可见分区：

- `common`
- `input`
- `process`
- `output`

`common` 是资产、profile、参数包和被动初始化数据层，不是隐藏的运行时物理层。
`input`、`process`、`output` 是正式运行阶段，分别对应测量侧输入、制导控制过程逻辑、
以及推进、气动、质量变化等运行时效应。

这个边界的目的很直接：避免为了兼容旧例子把物理运行时组件塞回 `common`，
也避免把示例探针、测试夹具一类组件提升成 framework builtin API。

## 3. 服务由 service package 声明作用域

服务不是散落在 `MissionAssembler` 中的特殊分支，而是通过
`ServicePackageRegistry` 注册的服务包。服务包负责声明：

- service id
- 支持的作用域
- service 实例创建逻辑
- 需要组件注册完成后执行的 finalization task

`buildServices()` 只做通用流程：查找服务包、检查作用域、创建服务、收集 finalization task。
具体服务语义必须留在服务包或服务自己的 bootstrap 中。

当前支持的服务作用域模型是：

- `global_services`
- `environment.services`
- `vehicle.services`

单个服务可以只支持其中一部分作用域。对不支持的作用域，框架应明确拒绝，
而不是在 generic assembler 中写死某个示例的行为。

## 4. `coordinate_tree` 是 vehicle-scoped v1 服务

`coordinate_tree` 当前版本只支持 `vehicle.services.coordinate_tree`。
`global_services.coordinate_tree` 和 `environment.services.coordinate_tree`
会被拒绝，这是当前设计选择，不是临时兼容缺口。

原因：

- 当前已实现的 frame tree 绑定 vehicle-local truth、launch、track 等语义
- `ICoordService` 的主要消费者是飞行器侧过程、输出和诊断组件
- global/environment 范围的坐标树需要先定义跨 vehicle truth、共享 frame 命名和生命周期规则

如果未来需要全局或环境坐标树，应作为新的服务作用域设计推进，而不是放宽当前校验。

## 5. Coordinate-tree specs 由服务包内置管理

`ICoordinateTreeSpec` 不是项目侧自动注册扩展点。当前设计是：

- coordinate-tree specs 由 coordinate-tree 服务包拥有
- 新 spec 放在 `framework/include/gnc/services/coordinate_tree/specs/`
- spec 注册入口位于 coordinate-tree 服务 bootstrap
- `SimulationBuilder` 和 core assembler 不包含具体 spec 清单

这样可以把坐标系语义留在 coordinate-tree 服务边界内，避免每新增一个兼容 spec
都扩大 core 的编译期依赖和架构职责。

## 6. `SimulationBuilder` 只做高层编排

当前构建职责拆分为：

| 类型 | 职责 |
| --- | --- |
| `SimulationBuilder` | 读取 mission、设置仿真参数和积分器、触发装配、验证、停机条件和 logger 初始化、汇总诊断 |
| `MissionAssembler` | 按当前 schema 装配 global/environment/vehicle/interaction，调用 package registry，注册组件接口 |
| `ServicePackageRegistry` | 保存 service package 清单，隔离服务创建与 finalization 逻辑 |
| `ValidationPipeline` | 执行 build 期依赖预检，产出验证错误和警告 |
| `StopConditionBuilder` | 解析 stop conditions，查找字段并注册终止条件 |

这个边界防止 `SimulationBuilder` 重新膨胀成“什么都知道”的中心类。
core 可以依赖抽象 registry 和 builtin bootstrap，但不能直接积累具体坐标树 spec、
示例组件或服务内部语义。

## 7. 运行循环使用固定步长

当前仿真循环围绕固定步长 `simulation.dt` 组织。它同时驱动：

- 积分器
- 组件调度
- 自动记录器
- 停机条件检查

这样做的好处是记录数据稳定、组件频率换算直接、用户容易理解每一步发生的事情。

代价是当前不支持可变步长积分器。若以后引入自适应步长，需要同时重构积分、
调度、日志采样和停机条件语义，而不是只改积分器接口。

## 8. 注册模型以显式 bootstrap 为准

当前框架不依赖运行时动态发现或静态注册兜底路径。

正式注册入口是：

- framework builtins 通过显式 bootstrap 函数注册
- service builtins 通过 service package bootstrap 注册
- 项目组件通过构建系统生成的项目注册入口纳入编译

示例组件和测试夹具不属于稳定 builtin API。为了让旧示例或测试通过而把 demo
组件注册进 framework builtin，会污染架构边界，应避免。

## 9. 当前文档基线高于历史实施细则

正式真源是：

- `README.md`
- `doc/`

`temp/seeByAgent/*` 可以作为历史评审材料，但不再约束当前实现。
当历史测试、旧示例或旧文档与当前架构冲突时，应优先更新测试和文档，
而不是在 framework 中恢复兼容分支。
