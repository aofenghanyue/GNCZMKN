# User Workspace

`user/` 是项目侧工作区，用来放 mission、项目组件、项目私有头文件、共享数据资产和仿真输出。framework 代码不应依赖某个具体 `user/<project>`。

## 目录结构

| Path | Purpose |
| --- | --- |
| `user/active_project` | 未设置 `-DGNC_ACTIVE_PROJECT=...` 时的默认 active project 名 |
| `user/<project>/config/mission.json` | active project 的默认 mission |
| `user/<project>/components/` | 会被 CMake 扫描并自动注册的项目组件头文件 |
| `user/<project>/interfaces/` | 项目私有接口；进入 include path，不扫描注册 |
| `user/<project>/common/` | 项目共享工具；进入 include path，不扫描注册 |
| `user/<project>/include/` | 项目私有 include root；进入 include path，不扫描注册 |
| `user/data/` | 跨项目数据资产，可用 `user-data://...` 引用 |
| `user/outputs/` | 运行时输出目录 |

`components/` 只放包含 `GNC_REGISTER_COMPONENT_TYPE` 的可注册组件头。共享接口、数据结构、工具函数应放在 `interfaces/`、`common/` 或 `include/`。

## Active Project

CMake 按以下顺序选择 active project：

1. `-DGNC_ACTIVE_PROJECT=<project>` cache value。
2. `user/active_project`。

如果两者都存在且不同，CMake 会 warning，并使用 cache value。active project 必须提供 `config/mission.json`；否则 CMake 配置失败。如果没有 active project，`gnc_sim` 没有默认 mission，运行时必须传 `--config <path>`。

当前仓库默认：

```text
example_02_atmospheric_3dof
```

只有 active project 的 `components/*.hpp` 会自动注册进 `gnc_sim`。修改 `user/active_project`、`-DGNC_ACTIVE_PROJECT` 或组件头文件后，需要重新运行 CMake 配置并重新构建。

## 示例项目

| Project | Purpose |
| --- | --- |
| `example_01_minimal_pluginized` | 最小 Cartesian 3DoF mission |
| `example_02_atmospheric_3dof` | 主 atmospheric local-spherical 3DoF 示例 |
| `example_03_coordinate_tree` | coordinate-tree service 和项目组件示例 |

## 常用命令

运行 active project mission：

```powershell
build-mingw\bin\gnc_sim.exe
```

运行指定 mission：

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

查看已注册组件：

```powershell
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

切换 active project 的推荐方式：

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DGNC_ACTIVE_PROJECT=example_03_coordinate_tree
cmake --build build-mingw -j 4
```
