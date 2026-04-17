# 文档首页

这套文档面向两类读者：要跑任务、改任务、加组件的使用者，以及要维护框架边界、插件和装配流程的开发者。阅读顺序按“先能用，再能扩展，最后理解内部结构”组织。

## 推荐阅读顺序

| 顺序 | 文档 | 适合解决的问题 |
| --- | --- | --- |
| 1 | [快速上手](01-getting-started.md) | 如何构建、运行、定位输出 |
| 2 | [核心概念](02-core-concepts.md) | 框架里哪些对象负责什么 |
| 3 | [任务配置](03-mission-configuration.md) | 如何写 `mission.json` |
| 4 | [扩展指南](04-extension-guide.md) | 如何加项目组件、内置组件或服务 |
| 5 | [架构说明](05-architecture.md) | 装配流程、运行时循环和模块依赖方向 |
| 6 | [参考手册](06-reference.md) | 查命令、字段、内置组件和命名规则 |
| 7 | [设计决策](07-decisions.md) | 为什么当前架构这样划分 |

如果只是运行示例任务，读前两篇就够。如果要新增 GNC 算法组件，读到扩展指南。如果要改内核或插件边界，需要读架构说明和设计决策。

## 术语约定

| 中文术语 | 代码名 | 含义 |
| --- | --- | --- |
| 组件 | `Component` | 仿真中被调度的最小业务单元，继承 `ComponentBase` |
| 插件 | `Plugin` | 编译期注册的一组内置组件或服务安装器 |
| 实体 | `Entity` | 任务配置里的顶层装配单元，当前角色为 `environment` 或 `vehicle` |
| 飞行器实体 | `vehicle` | 一个可独立配置组件和局部服务的飞行器 |
| 环境实体 | `environment` | 全任务共享的环境组件集合，当前只支持一个 |
| 服务 | `Service` | 非组件能力，存放在 `ServiceContext`，由插件安装 |
| 注册表 | `ComponentRegistry` | 保存组件实例和接口映射 |
| 作用域注册表 | `ScopedRegistry` | 带实体作用域的依赖查找视图 |
| 可观测字段 | `IObservable` | 自动记录器可以写入 CSV 的稳定字段 |
| 连续系统 | `IContinuousSystem` | 由积分器推进状态的组件 |

文档里会保留英文代码标识。比如“组件”对应 `ComponentBase`，但不会把 `ComponentBase` 翻译成“组件基类对象”。配置键也保留英文，比如 `entities`、`global_services`、`outputs.record`。

## 写配置时先记住三件事

- 所有组件都放在 `entities[]` 里。
- 环境组件的完整名称是 `env.<component_name>`。
- 飞行器组件的完整名称是 `<entity_id>.<component_name>`。

日志、停止条件和跨实体绑定应使用完整名称。组件代码里的同实体依赖可以使用局部名，由 `ScopedRegistry` 自动补作用域。

