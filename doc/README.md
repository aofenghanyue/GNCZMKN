# 文档导航

这些文档描述当前 GNCZMKN 运行时契约。若文档、代码和测试不一致，以代码和测试为准，并优先修正文档。

优秀的仿真框架文档不只列 API。它应回答四类问题：模型边界是什么、mission 如何稳定复现、扩展点怎样接入、输出数据怎样解释。本文档集按这个标准组织。

## 读者路径

| 你要做什么 | 先读 | 再读 |
| --- | --- | --- |
| 第一次构建和运行 | [01-getting-started.md](01-getting-started.md) | [../user/README.md](../user/README.md) |
| 理解框架全貌 | [02-core-concepts.md](02-core-concepts.md) | [00-current-architecture.md](00-current-architecture.md), [05-architecture.md](05-architecture.md), [04-extension-guide.md](04-extension-guide.md) |
| 写或审查 mission | [02-core-concepts.md](02-core-concepts.md) | [03-mission-configuration.md](03-mission-configuration.md), [06-reference.md](06-reference.md) |
| 使用拉偏和 RunSet | [03-mission-configuration.md](03-mission-configuration.md) | [02-core-concepts.md](02-core-concepts.md), [04-extension-guide.md](04-extension-guide.md), [06-reference.md](06-reference.md) |
| 新增项目组件 | [04-extension-guide.md](04-extension-guide.md) | [03-mission-configuration.md](03-mission-configuration.md), [../user/README.md](../user/README.md) |
| 维护 framework | [00-current-architecture.md](00-current-architecture.md) | [05-architecture.md](05-architecture.md), [07-decisions.md](07-decisions.md) |

## 示例 Mission

| 示例 | 用途 |
| --- | --- |
| `user/example_04_ideal_6dof_baseline/config/mission.json` | ideal local-spherical 6DoF baseline flow |
| `user/example_05_ideal_3dof_geographic_baseline/config/mission.json` | ideal local-spherical 3DoF geographic baseline flow |
| `user/example_06_ideal_cartesian_3dof_baseline/config/mission.json` | ideal launch-frame Cartesian 3DoF baseline flow |
| `user/example_07_ideal_cartesian_6dof_baseline/config/mission.json` | ideal launch-frame Cartesian 6DoF baseline flow |
| `user/example_08_cavh_geographic_3dof_custom/config/mission.json` | CAVH custom geographic 3DoF user project |

## 文档质量标准

- **先给心智模型**：读者应先理解 `Form / Environment / Vehicle / Interaction` 的边界，再看 type id。
- **配置可复制**：关键 JSON 片段应能直接映射到示例 mission，避免只给抽象 schema。
- **时间语义清楚**：记录、停止条件、`update()`、`publish()` 和积分提交的顺序必须明确。
- **扩展有检查清单**：新增组件时应能检查 role、stage、form family、接口和 placement。
- **参考手册完整**：CLI、内置组件、服务、输出字段和常见配置字段要集中索引。

## 文档文件

- [00-current-architecture.md](00-current-architecture.md)：全局架构地图、mission build 主链路、runtime 数据流。
- [01-getting-started.md](01-getting-started.md)：构建、测试、运行、查看输出。
- [02-core-concepts.md](02-core-concepts.md)：核心心智模型、数据流概念、作用域、调度和记录语义。
- [03-mission-configuration.md](03-mission-configuration.md)：mission schema、include、vehicles、拉偏机制、服务、outputs、termination、summary。
- [04-extension-guide.md](04-extension-guide.md)：active project 组件、注册元数据、自定义 interaction、拉偏组件/使用者契约、修改类型决策表。
- [05-architecture.md](05-architecture.md)：维护者视角的源码导览、RunSet 执行模型、关键类职责、生命周期和调度机制。
- [06-reference.md](06-reference.md)：CLI、RunSet 配置、源码入口、type id、服务 id、配置字段速查。
- [07-decisions.md](07-decisions.md)：当前架构决策记录。
