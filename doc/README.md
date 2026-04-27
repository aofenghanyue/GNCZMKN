# 文档导航

这组文档描述当前 GNCZMKN 运行时和用户契约。阅读时优先相信代码和测试；公开文档的目标是把这些契约整理成可入门、可扩展、可查阅的说明。

## 第一次运行

先读：

- [01-getting-started.md](01-getting-started.md): 安装前提、MinGW 构建、运行默认任务、指定 mission、查看输出和常见故障。
- [user/README.md](../user/README.md): `user/active_project`、示例项目、项目组件和输出目录的关系。

完成后应能做到：

- 构建 `build-mingw\bin\gnc_sim.exe`。
- 跑通 `user/example_02_atmospheric_3dof/config/mission.json`。
- 找到 `user/outputs/{timestamp}` 下的 CSV 和仿真摘要。

## 理解框架

按这个顺序读：

- [00-current-architecture.md](00-current-architecture.md): 当前架构的短总览。
- [02-core-concepts.md](02-core-concepts.md): `Form / Environment / Vehicle / Interaction`、调度顺序、命名作用域、资产与运行时组件的边界。
- [05-architecture.md](05-architecture.md): 面向维护者的装配、服务、验证、日志和调度数据流。

## 编写 Mission

主要入口：

- [03-mission-configuration.md](03-mission-configuration.md): mission 顶层结构、vehicle 分层、输出、停止条件和 `coordinate_tree` 服务。
- [06-reference.md](06-reference.md): CLI、builtin type id、服务 id、常用字段和命名规则速查。

建议从这三个示例对照阅读：

- `user/example_01_minimal_pluginized/config/mission.json`: 当前最小 Cartesian 3DoF 示例，目录名保留历史痕迹。
- `user/example_02_atmospheric_3dof/config/mission.json`: 当前推荐的 atmospheric local-spherical 3DoF 主示例。
- `user/example_03_coordinate_tree/config/mission.json`: `vehicles[].services.coordinate_tree` 示例。

## 新增组件

主要入口：

- [04-extension-guide.md](04-extension-guide.md): active project、`user/<project>/components/`、注册宏、role/stage/form-family、project component 与 builtin 的选择。
- [user/components/README.md](../user/components/README.md): 旧 `user/components/` 目录为什么不再作为推荐扩展入口。

## 维护架构和决策

维护 framework 内部时阅读：

- [05-architecture.md](05-architecture.md): 装配路径和运行时边界。
- [07-decisions.md](07-decisions.md): 当前仍有效的设计决策。

历史迁移材料可以保留在 `temp/` 中供追溯，但不作为新用户入口，也不作为当前兼容目标。
