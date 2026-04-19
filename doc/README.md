# 文档首页

这套文档面向两类读者：

- 要跑 mission、改 mission、加组件的使用者
- 要维护框架边界、插件和装配流程的开发者

阅读顺序按“先能用，再能扩展，最后理解内部结构”组织。

## 推荐阅读顺序

| 顺序 | 文档 | 适合解决的问题 |
| --- | --- | --- |
| 1 | [快速上手](01-getting-started.md) | 如何构建、运行、定位输出 |
| 2 | [核心概念](02-core-concepts.md) | 框架里哪些对象负责什么 |
| 3 | [任务配置](03-mission-configuration.md) | 如何写 `mission.json` |
| 4 | [扩展指南](04-extension-guide.md) | 如何加项目组件、服务或内置插件 |
| 5 | [架构说明](05-architecture.md) | 装配流程、运行时循环和模块边界 |
| 6 | [参考手册](06-reference.md) | 查命令、字段、组件类型和命名规则 |
| 7 | [设计决策](07-decisions.md) | 为什么当前架构这样取舍 |

如果只是运行示例 mission，读前 3 篇就够。若要新增 GNC 组件或修改装配流程，再继续看扩展指南和架构说明。

## 术语约定

| 中文术语 | 代码名 | 含义 |
| --- | --- | --- |
| 组件 | `Component` | 仿真中被调度的最小业务单元，继承 `ComponentBase` |
| 插件 | `Plugin` | 编译期注册的一组内置组件或服务安装器 |
| 实体 | `Entity` | mission 顶层装配单元，当前角色为 `environment` 或 `vehicle` |
| 服务 | `Service` | 非组件能力，存放在 `ServiceContext`，由插件安装 |
| 注册表 | `ComponentRegistry` | 保存组件实例和接口映射 |
| 作用域注册表 | `ScopedRegistry` | 带实体作用域的依赖查找视图 |
| 稳定字段 | `IObservable` | 自动记录器可以稳定写出的字段 |
| 调试快照 | `snapDebug()` | 只用于临时排障的逐步调试值 |
| 连续系统 | `IContinuousSystem` | 由积分器推进状态的组件 |

## 写配置前先记住三件事

- 所有组件都放在 `entities[]` 里。
- 环境组件完整名是 `env.<component_name>`。
- 飞行器组件完整名是 `<entity_id>.<component_name>`。

补充两条正式基线：

- 旧式根级 `components` / `services` / `vehicles` 已不支持。
- 单飞行器也不再保留空前缀命名。
