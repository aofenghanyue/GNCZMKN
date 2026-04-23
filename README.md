# GNCZMKN

GNCZMKN 是一个面向制导、导航与控制工作的 C++ 仿真框架。当前版本的核心目标是让仿真任务用清晰的架构块装配，而不是依赖旧式隐式装配或实体列表。

当前框架围绕四个顶层概念组织：

- `Form`: 状态表示、积分方程、truth view 和 form input。
- `Environment`: 地球、重力、大气等环境查询能力。
- `Vehicle`: 飞行器侧组件，按 `common/input/process/output` 分层。
- `Interaction`: 把 form truth、环境查询、过程命令和 vehicle output 能力闭合成 form input。

每个仿真步的执行顺序固定为：

1. `environment`
2. `vehicle.input`
3. `vehicle.process`
4. `vehicle.output`
5. `interaction`
6. `form`
7. 自动记录和停止条件检查

## 快速开始

Windows 下推荐使用 MinGW 构建目录：

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

运行当前 active project 的默认任务：

```powershell
build-mingw\bin\gnc_sim.exe
```

显式运行一个示例任务：

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

查看当前构建注册了哪些组件：

```powershell
build-mingw\bin\gnc_sim.exe --list-components
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

`user/active_project` 选择会被编译进 `gnc_sim` 的项目组件。修改 active project 或新增项目组件后，需要重新运行 CMake 配置并重新构建。

## Mission 形状

当前 mission 使用显式顶层块：

```text
simulation:
form:
environment:
vehicle:
interaction:
outputs:
stop_conditions:
global_services:
```

`vehicle` 内部按职责拆分：

```text
vehicle:
  services:
  common:
  input:
  process:
  output:
```

其中 `vehicle.common` 是静态资产和 profile 层，不参与运行时调度；`input/process/output` 是正式运行阶段。旧式 `entities[]`、根级 `components`、根级 `services` 和根级 `vehicles` 不属于当前运行时契约。

## 仓库结构

| 路径 | 用途 |
| --- | --- |
| `framework/include/gnc/core/` | 运行时外壳、mission 装配、验证、日志、停止条件 |
| `framework/include/gnc/forms/` | 内置 form 包和 form 专属接口 |
| `framework/include/gnc/environment/` | 环境组件和环境查询接口 |
| `framework/include/gnc/vehicle/` | vehicle common/input/process/output 包 |
| `framework/include/gnc/interactions/` | form-aware interaction 包 |
| `framework/include/gnc/services/` | service package 和 service 侧契约 |
| `framework/data/` | 仿真运行时可加载资产 |
| `src/runner.cpp` | 命令行入口 |
| `user/` | active project、示例 mission、项目组件和输出 |
| `tests/` | 架构契约、组件注册、mission 装配和日志测试 |
| `doc/` | 当前用户文档和维护者文档 |

## 文档入口

从这里开始：

- [文档导航](doc/README.md)
- [快速上手](doc/01-getting-started.md)
- [核心概念](doc/02-core-concepts.md)
- [Mission 配置](doc/03-mission-configuration.md)
- [扩展指南](doc/04-extension-guide.md)
- [参考手册](doc/06-reference.md)

维护架构时再阅读：

- [当前架构总览](doc/00-current-architecture.md)
- [维护者架构说明](doc/05-architecture.md)
- [设计决策](doc/07-decisions.md)
- [User 工作区说明](user/README.md)
