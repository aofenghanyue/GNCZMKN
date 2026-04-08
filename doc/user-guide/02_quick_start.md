# 02 快速开始

## 2.1 环境要求

从当前 `CMakeLists.txt` 可以直接读出构建前提：

- CMake 3.16 或更高
- 支持 C++17 的编译器
- Eigen3 3.3 或更高

推荐环境：

- Windows: Visual Studio 2022 或 MSVC Build Tools + CMake
- Linux: GCC 9+/Clang 12+ + CMake
- 可选: Python 3，用于后处理脚本

## 2.2 先安装哪些依赖

### 必选依赖

- Eigen3

如果 CMake 配置时报 `find_package(Eigen3 3.3 REQUIRED NO_MODULE)` 失败，需要先安装 Eigen3，再让 CMake 能找到它。

可选安装方式示例：

- Ubuntu/Debian: `sudo apt install libeigen3-dev`
- vcpkg: `vcpkg install eigen3`
- 手动安装后，通过 `CMAKE_PREFIX_PATH` 或工具链文件告知 CMake

## 2.3 标准构建命令

### Windows / 通用 CMake

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

说明：

- 第一条命令生成构建系统
- 第二条命令完成编译
- `gnc_sim` 默认输出到 `build/bin/gnc_sim.exe`

### 使用仓库自带脚本

仓库提供了 `tools/build_and_run.ps1`，它会：

- 自动创建 `build/`
- 运行 CMake 配置
- 编译工程
- 运行 `gnc_sim`

示例：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_and_run.ps1
```

指定任务文件：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_and_run.ps1 -ConfigFile user\config\missions\minimal.json
```

只构建不运行：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_and_run.ps1 -BuildOnly
```

## 2.4 自动注册机制是什么

构建时，顶层 `CMakeLists.txt` 会递归扫描：

- `user/components/**/*.hpp`

然后生成：

- `build/generated/user_components_register.hpp`

这个生成文件会被 `src/runner.cpp` 包含，从而把你的用户组件头文件引入编译单元。只要你的组件头文件末尾使用了注册宏，框架就能在运行前自动识别它。

这也是为什么添加用户组件通常不需要修改：

- `main.cpp`
- `runner.cpp`
- `CMakeLists.txt`

## 2.5 先验证框架是否工作正常

### 查看可用组件

```powershell
.\build\bin\gnc_sim.exe --list-components
```

如果你还想看每个类型来自哪个头文件，可以使用：

```powershell
.\build\bin\gnc_sim.exe '--list-components-verbose'
```

在当前仓库里，实际运行会列出：

- Starter components，例如 `Wgs84Earth`、`SphericalGravity`、`StandardAtmosphere`、`SimpleDynamics`
- Custom/example components，例如用户组件 `ConstantGuidance`

这一步很重要，因为它能立刻验证：

- 组件是否成功注册
- 你的自定义头文件是否被构建系统发现
- 你看到的类型到底属于“仓库随附起步件”还是“你自己的扩展件”
- verbose 模式下还能直接确认某个类型的注册来源文件

### 运行现成的最小任务

```powershell
.\build\bin\gnc_sim.exe --config user\config\missions\minimal.json
```

当前仓库已经内置了可运行的最小配置，它会装配：

- 地球模型
- 常值制导
- 简化动力学
- 理想 IMU
- 简单导航

它适合验证整个主链是否通了。

## 2.6 如何运行示例

### 直接运行默认入口

如果不传参数，`gnc_sim` 默认读取：

- `user/config/missions/default.json`

运行器会从：

- 当前工作目录
- 可执行文件所在目录

向上搜索这条相对路径，所以你既可以在仓库根目录运行，也可以直接在 `build/bin` 下运行默认入口。
对这类 repo-relative mission，`outputs.directory` 这类相对路径也会继续落在同一个项目根下，而不是掉到 `build/bin` 下面。

当前的 `default.json` 已调整为一个可直接跑通的最小闭环任务，适合作为首次启动入口。它会自动装配：

- `ConstantGuidance`
- `SimpleDynamics`
- `IdealImu`
- `SimpleNavigation`

并在 `user/outputs/{timestamp}` 下生成默认输出。

### 运行 CAV-H 3DOF 示例

该示例单独编译为可执行文件：

```powershell
.\build\bin\example_cavh_3dof.exe
```

默认会读取：

- `examples/03_cavh_3dof/cavh_mission.json`

如果你想用统一入口运行它，也可以直接执行：

```powershell
.\build\bin\gnc_sim.exe --config user\config\missions\minimal.json
```

这里要注意一条边界：

- `--config` 负责的是 mission 路径解析
- 它不会自动把 `examples/` 目录下的专用组件注册进 `gnc_sim`

因此像 `examples/03_cavh_3dof` 这类自带专用组件的示例，仍应优先运行它自己的示例可执行文件，而不是假设统一入口会自动获得 example 组件注册能力。

## 2.7 输出文件会出现在哪里

如果任务配置中包含 `outputs` 段，框架会自动创建输出目录。当前示例通常写到：

- `user/outputs/{timestamp}/`

典型输出包括：

- `*.csv`：记录的可观测数据
- `summary.txt`：仿真摘要，包括步长、执行时间、终止原因、组件实例列表，以及本次任务实际使用的 starter/custom 类型清单

如果没有配置 `outputs`，仿真仍然可以运行，只是不会自动写 CSV。

## 2.8 第一次做自定义实验的推荐路径

最省力的方式不是从零开始写新模型，而是按下面顺序：

1. 先运行 `user/config/missions/minimal.json`
2. 阅读 `user/components/guidance/constant_guidance.hpp`
3. 调整制导参数，验证输出变化
4. 再决定是替换制导、动力学、导航，还是增加新环境组件

这样做的原因是：

- 你先确认了构建系统、自动注册、JSON 配置和主循环都可用
- 后续每次改动只需要聚焦一个层面

## 2.9 常见上手问题

### CMake 找不到 Eigen3

说明依赖没装好，或 CMake 没有找到安装位置。优先处理环境问题，不要先怀疑框架代码。

### `Unknown component type`

说明 mission 里的 `type` 与注册名不一致，或者用户头文件没有被扫描到。

检查顺序：

1. 组件头文件是否放在 `user/components/` 下
2. 头文件末尾是否写了注册宏
3. `--list-components` 能否看到该类型名
4. JSON 里的 `type` 是否与类名/注册名完全一致

### 运行了但没有 CSV

通常有三种原因：

- 没写 `outputs`
- 写了 `outputs` 但没有任何组件实现 `IObservable`
- `outputs.record` 规则没有匹配到字段

### 有输出目录但字段为空

优先检查：

- 组件是否实现了 `IObservable`
- 字段名是否和 `record` 配置一致
- `exclude` 是否把字段过滤掉了
