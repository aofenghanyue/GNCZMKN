# 快速上手

这篇文档只处理第一次使用要完成的事情：构建、测试、运行一个 mission、确认组件注册、找到输出。

## 环境要求

- CMake 3.16 或更新版本。
- 支持 C++17 的 MinGW 工具链。
- Eigen3，且能被 `find_package(Eigen3 3.3 REQUIRED NO_MODULE)` 找到。

Windows 下推荐使用 MinGW 生成器。不要在同一个构建目录里混用 Visual Studio 生成器和 MinGW 生成器。

## 构建和测试

在仓库根目录运行：

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

`build-mingw` 是推荐构建目录。切换编译器或生成器时，新建另一个构建目录更稳妥。

## 运行默认任务

直接运行：

```powershell
build-mingw\bin\gnc_sim.exe
```

默认任务由构建时的 active project 决定。当前 active project 写在：

```text
user/active_project
```

如果 active project 存在 `config/mission.json`，它会成为默认任务。当前仓库默认选择：

```text
example_02_atmospheric_3dof
```

对应 mission：

```text
user/example_02_atmospheric_3dof/config/mission.json
```

修改 `user/active_project` 或新增项目组件后，重新运行 CMake 配置和构建。

## 运行指定任务

可以显式传入 mission 路径：

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

也可以省略 `--config`：

```powershell
build-mingw\bin\gnc_sim.exe user/example_01_minimal_pluginized/config/mission.json
```

相对路径可以相对当前工作目录，也可以相对仓库根目录。runner 会从当前目录和可执行文件目录向上搜索。

## 查看组件注册

查看 type id：

```powershell
build-mingw\bin\gnc_sim.exe --list-components
```

查看 type id、接口、role、stage、form family 和注册来源：

```powershell
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

新增项目组件后，先用这个命令确认 type id 是否进入当前构建。如果没有出现，通常需要检查：

- 组件头文件是否在 `user/<active_project>/components/` 下。
- `GNC_REGISTER_COMPONENT_TYPE` 的 type id 是否与 mission 中的 `type` 完全一致。
- 是否重新运行了 CMake 配置和构建。

## 输出位置

输出由 mission 的 `outputs` 块控制。例如：

```json
{
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "cavh_3dof",
    "record": {
      "cavh.dynamics": "all"
    }
  }
}
```

`{timestamp}` 会在运行时替换为时间戳。自动记录器会写 CSV；仿真结束后，如果输出目录有效，还会写仿真摘要。

## 常用命令

```powershell
build-mingw\bin\gnc_sim.exe --help
build-mingw\bin\gnc_sim.exe --list-components
build-mingw\bin\gnc_sim.exe --list-components-verbose
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

## 常见问题

| 现象 | 处理方式 |
| --- | --- |
| CMake 找不到 Eigen3 | 确认 Eigen3 安装位置，并通过工具链文件或 `CMAKE_PREFIX_PATH` 暴露给 CMake |
| 默认任务找不到 | 使用 `--config <path>` 显式传入 mission，或检查 `user/active_project` |
| 修改 active project 后组件清单没变 | 重新运行 CMake 配置，再重新构建 |
| 提示未知组件类型 | 检查 mission 的 `type` 是否等于注册的 type id |
| 组件放错块 | 检查 type 的 role 和 stage，`--list-components-verbose` 会列出注册元数据 |
| 没有生成 CSV | 检查 `outputs.enabled`、`outputs.record`，并确认目标组件实现 `IObservable` |
| 停止条件找不到字段 | 确认 `component` 使用完整组件名，例如 `cavh.dynamics` |

下一步建议阅读 [核心概念](02-core-concepts.md)，再阅读 [Mission 配置](03-mission-configuration.md)。
