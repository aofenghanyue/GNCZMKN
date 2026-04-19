# GNCZMKN

GNCZMKN 是一个面向制导、导航与控制仿真的 C++17 框架。它把任务配置、实体、组件、服务和数据记录拆开，让同一套内核可以装配不同 mission。

当前框架采用编译期注册。

- 内置组件由插件注册。
- 项目组件放在 `user/<project>/components/` 下，由 CMake 自动纳入构建。
- 运行时不扫描动态库。

## 快速开始

Windows 下推荐使用 MinGW CMake 配置：

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

运行当前活动项目：

```powershell
build-mingw\bin\gnc_sim.exe
```

运行指定 mission：

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

查看已注册组件：

```powershell
build-mingw\bin\gnc_sim.exe --list-components
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

`--list-components-verbose` 会同时显示 type id、接口列表和注册来源。

## 目录结构

| 路径 | 作用 |
| --- | --- |
| `framework/include/gnc/core/` | 仿真内核、注册表、构建编排、验证与停机条件构建 |
| `framework/include/gnc/plugins/` | 内置插件与内置组件 |
| `src/runner.cpp` | 命令行入口 |
| `user/<project>/components/` | 项目自定义组件 |
| `user/<project>/config/mission.json` | 项目 mission 配置 |
| `doc/` | 框架文档 |

## 文档入口

| 文档 | 内容 |
| --- | --- |
| [文档首页](doc/README.md) | 阅读顺序和术语约定 |
| [快速上手](doc/01-getting-started.md) | 构建、运行、输出和常见问题 |
| [核心概念](doc/02-core-concepts.md) | Component、Plugin、Entity、Service、Registry |
| [任务配置](doc/03-mission-configuration.md) | `entities[]`、服务、停机条件、输出配置 |
| [扩展指南](doc/04-extension-guide.md) | 添加项目组件、连续系统、服务安装器、内置插件扩展 |
| [架构说明](doc/05-architecture.md) | 装配流程、运行循环和模块边界 |
| [参考手册](doc/06-reference.md) | 命令、字段、组件类型、命名规则 |
| [设计决策](doc/07-decisions.md) | 当前架构的关键取舍 |

## 当前基线

- 顶层 mission 只支持 `entities[]`。
- 旧式根级 `components` / `services` / `vehicles` 已正式废弃，不再兼容。
- 环境组件完整名固定为 `env.<component_name>`。
- 飞行器组件完整名固定为 `<entity_id>.<component_name>`，单飞行器和多飞行器不再分叉。
- `SimulationBuilder` 负责高层编排，`MissionAssembler` 负责实体装配，`ValidationPipeline` 负责依赖预检和诊断，`StopConditionBuilder` 负责停机条件解析。
- 运行时使用固定步长仿真，当前支持 `rk4` 和 `euler`。

## 与历史实施细则的差异

- 不再兼容旧式根级 mission。
- 不再保留单飞行器空前缀命名。
