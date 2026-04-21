# 快速上手

这篇文档只处理一件事：把项目构建起来，跑通一个任务，并知道输出在哪里。

## 环境要求

- CMake 3.16 或更新版本。
- 支持 C++17 的 MinGW 工具链。
- Eigen3，且能被 `find_package(Eigen3 3.3 REQUIRED NO_MODULE)` 找到。

在 Windows 下请使用 MinGW 生成器，不要混用 Visual Studio 生成器和 MinGW 构建目录。

## 构建和测试

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

`build-mingw` 是推荐构建目录。需要重新配置时，可以继续复用这个目录；如果你换了编译器或生成器，应该新建另一个构建目录。

## 运行任务

直接运行会使用活动项目的默认任务：

```powershell
build-mingw\bin\gnc_sim.exe
```

活动项目写在：

```text
user/active_project
```

如果活动项目存在 `config/mission.json`，它就是默认任务。也可以显式指定任务：

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

可用命令：

```powershell
build-mingw\bin\gnc_sim.exe --help
build-mingw\bin\gnc_sim.exe --list-components
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

`--list-components` 会列出当前构建里注册成功的内置组件和项目组件。新增项目组件后，先看这里能不能看到新的 type id。

## 输出位置

任务通过 `outputs.directory` 指定输出目录。示例任务通常使用：

```json
{
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "atmospheric_3dof"
  }
}
```

`{timestamp}` 会在运行时替换成时间戳。自动记录器会写 CSV 数据；仿真结束后，如果输出目录有效，还会写仿真摘要。

## 常见问题

| 现象 | 处理方式 |
| --- | --- |
| CMake 找不到 Eigen3 | 先确认 Eigen3 安装位置，再通过 CMake 工具链或 `CMAKE_PREFIX_PATH` 暴露给 `find_package` |
| 默认任务找不到 | 使用 `--config <path>` 显式传入任务文件，或检查 `user/active_project` |
| 新组件没出现在列表里 | 确认组件头文件位于 `user/<active_project>/components/`，并重新运行 CMake 配置 |
| 提示未知组件类型 | 检查 `mission.json` 里的 `type` 是否与注册宏或插件注册的 type id 完全一致 |
| 依赖预检失败 | 检查组件名作用域。跨实体引用应写完整名，如 `env.atmosphere`、`missile.dynamics` |
| 没有生成 CSV | 检查 `outputs.enabled`、`outputs.record`，并确认目标组件实现了 `IObservable` |

下一步建议阅读 [核心概念](02-core-concepts.md)。
> Archived note: this document describes the removed plugin-era repository layout.
> For the current architecture and active mission shape, use
> [00-current-architecture.md](00-current-architecture.md).

