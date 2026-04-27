# 设计决策

这篇文档记录当前仍约束实现和文档的架构决策。它不是迁移日志，也不为旧实现保留兼容目标。

## ADR-001: Mission 使用显式架构块

**Status:** Accepted

当前 mission 顶层围绕下列块组织：

- `simulation`
- `form`
- `environment`
- `vehicle`
- `interaction`
- `outputs`
- `stop_conditions`
- `global_services`

这个结构直接暴露运行时职责边界。`form` 拥有状态，`environment` 提供环境能力，`vehicle` 组织飞行器侧组件，`interaction` 负责闭合成 form input。

决策：

- 不支持旧式 `entities[]`。
- 不支持旧式根级 `components/services/vehicles`。
- 不为了历史示例恢复旧 schema 分支。

## ADR-002: Vehicle 分成 common/input/process/output

**Status:** Accepted

`vehicle` 内部使用四个用户可见分区：

- `common`
- `input`
- `process`
- `output`

决策：

- `common` 是静态资产、profile、参数包和被动初始化数据层。
- `input` 是测量侧输入层。
- `process` 是导航、制导、控制和命令生成层。
- `output` 是气动、质量、推进、分离和构型切换等运行时效应层。
- `common` 不参与 stage 调度，也不能承载运行时物理行为。

## ADR-003: 资产、Loader 和 Runtime Component 分离

**Status:** Accepted

资产文件、loader/parser 和 runtime component 是三个不同职责。

决策：

- 资产文件放在 `framework/data/` 或项目数据目录中。
- loader/parser 是工具代码，不注册为组件。
- runtime component 暴露仿真时接口，并按 role/stage 调度。
- 气动和质量等运行时能力属于 `vehicle.output`。

## ADR-004: Interaction 保持 form-aware closure

**Status:** Accepted

`Interaction` 的职责是把 form truth、environment 查询、process 命令和 output 能力闭合成 form input。

决策：

- interaction 不拥有气动表、质量定义或推进模型。
- interaction 可以消费 `IAeroModel`、`IConstantMass`、`IContinuousMass` 等 output 接口。
- form-specific interaction 必须声明 form family。

## ADR-005: 服务由 Service Package 声明作用域

**Status:** Accepted

服务通过 `ServicePackageRegistry` 注册，不在 `MissionAssembler` 中为每个服务写特殊装配分支。

决策：

- service package 声明 service id、支持 scope、创建逻辑和 finalization task。
- generic assembler 只负责查找 package、校验 scope、创建服务、收集 finalization task。
- 服务内部语义留在 service package 边界内。

## ADR-006: Coordinate Tree v1 只支持 Vehicle Scope

**Status:** Accepted

当前 `coordinate_tree` 只支持：

```text
vehicles[].services.coordinate_tree
```

决策：

- `global_services.coordinate_tree` 会失败。
- `environment.services.coordinate_tree` 会失败。
- 当前 frame tree 语义绑定 vehicle-local truth、launch 和 track。
- 如果未来需要 global/environment coordinate tree，应先设计跨 vehicle truth、共享 frame 命名和生命周期。

## ADR-007: Coordinate-tree Specs 由服务包内置管理

**Status:** Accepted

`ICoordinateTreeSpec` 不是项目侧自动注册扩展点。

决策：

- specs 放在 `framework/include/gnc/services/coordinate_tree/specs/`。
- spec 注册入口位于 coordinate-tree service package bootstrap。
- `SimulationBuilder` 和 generic assembler 不包含具体 spec 清单。

## ADR-008: SimulationBuilder 只做高层编排

**Status:** Accepted

`SimulationBuilder` 负责高层流程，不积累具体组件、服务或示例语义。

职责边界：

| 类型 | 职责 |
| --- | --- |
| `SimulationBuilder` | 加载 mission、设置仿真参数、触发装配、验证、停止条件和 logger 初始化 |
| `MissionAssembler` | 按当前 schema 装配服务和组件 |
| `ServicePackageRegistry` | 保存 service package 并隔离服务创建逻辑 |
| `ValidationPipeline` | 执行 build 期契约检查和依赖预检 |
| `StopConditionBuilder` | 解析停止条件并绑定 observable 字段 |

## ADR-009: 当前运行循环使用固定步长

**Status:** Accepted

当前运行循环围绕 `simulation.dt` 组织。固定步长同时驱动：

- integrator
- stage update
- auto logger
- stop condition 检查

决策：

- 当前不支持可变步长 integrator。
- 引入自适应步长时，需要同时重新定义调度、日志采样和停止条件语义。

## ADR-010: 注册模型以显式 Bootstrap 为准

**Status:** Accepted

当前框架不依赖隐藏静态注册兜底路径。

决策：

- framework builtins 通过显式 bootstrap 注册。
- service builtins 通过 service package bootstrap 注册。
- project components 通过构建系统生成的 active project 注册入口纳入编译。
- 示例组件和测试夹具不属于稳定 builtin API。

## ADR-011: 当前文档只服务当前架构

**Status:** Accepted

公开文档应服务当前用户和维护者，不再把历史迁移材料作为入门路径或兼容目标。

决策：

- 顶层 README 和 `doc/` 是当前文档入口。
- 历史材料只作追溯，不约束当前 runtime。
- 当旧文档、旧示例或历史评审材料与当前代码冲突时，优先更新文档和测试来反映当前实现。
