# GNCZMKN

GNCZMKN 是一个面向制导、导航与控制仿真的 C++17 框架。它把任务配置、飞行器实体、环境实体、组件、服务和数据记录拆开，让同一套仿真内核可以装配不同任务。

当前框架采用编译期组件发现。内置组件由插件注册，项目组件放在 `user/<project>/components/` 下，由 CMake 自动纳入构建。运行时不加载动态库。

## 快速开始

项目在 Windows 下请使用 MinGW CMake 配置：

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

运行当前活动项目：

```powershell
build-mingw\bin\gnc_sim.exe
```

运行指定任务：

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

查看已注册组件：

```powershell
build-mingw\bin\gnc_sim.exe --list-components
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

活动项目由 `user/active_project` 指定。当前默认任务通常来自 `user/<active_project>/config/mission.json`。

## 目录结构

| 路径 | 作用 |
| --- | --- |
| `framework/include/gnc/core/` | 仿真内核、组件注册、服务上下文、任务装配 |
| `framework/include/gnc/plugins/` | 内置插件与内置组件 |
| `src/runner.cpp` | 命令行入口 |
| `user/<project>/components/` | 项目自定义组件 |
| `user/<project>/config/mission.json` | 项目任务配置 |
| `doc/` | 框架文档 |

## 文档入口

建议从这里开始：

| 文档 | 内容 |
| --- | --- |
| [文档首页](doc/README.md) | 阅读顺序和术语约定 |
| [快速上手](doc/01-getting-started.md) | 构建、运行、输出和常见问题 |
| [核心概念](doc/02-core-concepts.md) | Component、Plugin、Entity、Service、Registry 等概念 |
| [任务配置](doc/03-mission-configuration.md) | `entities[]`、单飞行器、多飞行器、服务和输出配置 |
| [扩展指南](doc/04-extension-guide.md) | 新增项目组件、内置组件和服务安装器 |
| [架构说明](doc/05-architecture.md) | 四层模型、装配流程和运行时循环 |
| [参考手册](doc/06-reference.md) | 命令、配置字段、组件类型、命名规则 |
| [设计决策](doc/07-decisions.md) | 当前架构的关键取舍 |

## 当前基线

- 内置插件：`environment`、`aero`、`state_3dof`、`soviet_coord`。
- 任务配置使用 entity-first 结构：顶层写 `entities[]`，不再把组件散放在根级字段里。
- 环境组件统一使用 `env.*` 名称，飞行器组件使用 `<entity_id>.*` 名称。
- `SimulationBuilder` 负责高层构建流程，`MissionAssembler` 负责实体、服务和组件装配。
- 仿真主循环使用固定步长，当前支持 `rk4` 和 `euler` 积分器。

