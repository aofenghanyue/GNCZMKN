# User 工作区

`user/` 是仓库内的用户项目工作区，用来放 mission、项目组件、active project 选择和仿真输出。

## 关键文件和目录

| 路径 | 用途 |
| --- | --- |
| `user/active_project` | 选择当前构建会自动注册哪个项目的组件 |
| `user/<project>/components/` | 当前推荐的项目组件目录 |
| `user/<project>/config/mission.json` | 项目默认 mission |
| `user/config/missions/default.json` | 没有 active project mission 时的 repository fallback mission |
| `user/outputs/` | 仿真输出目录 |
| `user/components/` | 旧过渡目录，不再作为推荐扩展入口 |

## Active Project

`user/active_project` 的内容是 `user/` 下的项目目录名。例如：

```text
example_02_atmospheric_3dof
```

CMake 配置时会读取这个文件，并扫描：

```text
user/<active_project>/components/
```

如果 active project 下存在：

```text
user/<active_project>/config/mission.json
```

它会成为 `gnc_sim.exe` 无参数运行时的默认 mission。

修改 `user/active_project` 后，需要重新运行 CMake 配置和构建。只改 mission JSON 时通常不需要重新构建，除非同时新增或删除了组件头文件。

## 当前示例

| 项目 | 用途 |
| --- | --- |
| `example_01_minimal_pluginized` | 当前最小 Cartesian 3DoF 示例。目录名保留历史痕迹，不代表当前架构命名 |
| `example_02_atmospheric_3dof` | 推荐主示例，覆盖 local-spherical 3DoF、environment、process、output、interaction 和 CSV 输出 |
| `example_03_coordinate_tree` | coordinate-tree 服务和项目组件示例，包含 `example.coordinate_probe` |

只有 active project 的项目组件会自动注册进 `gnc_sim`。例如从 `example_02_atmospheric_3dof` 切换到 `example_03_coordinate_tree` 后，需要重新配置和构建，`example.coordinate_probe` 才会出现在组件清单中。

## 常用流程

运行当前 active project 的默认 mission：

```powershell
build-mingw\bin\gnc_sim.exe
```

显式运行任意 mission：

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

查看当前构建包含哪些项目组件：

```powershell
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

## 输出目录

示例 mission 通常写入：

```text
user/outputs/{timestamp}
```

`{timestamp}` 由运行时替换。输出目录不应作为源代码或稳定测试输入依赖；需要保留结果时，应单独归档。
