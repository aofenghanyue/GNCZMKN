# GNC 仿真框架 - Code Wiki

> 本文档旨在帮助开发者（特别是新手）快速理解项目代码结构、核心设计理念，并能一步步搭建出自己的仿真模型。

## 1. 项目简介

本框架是一个面向 **GNC（制导、导航与控制）** 仿真的 C++17 组件化框架。其设计目标是：**让算法工程师只关心 GNC 策略本身**，无需操心坐标转换、积分器、数据记录、传感器仿真等基础设施。框架采用“接口查询+主动拉取”的组件间通信模式，并通过灵活的 JSON 配置实现单/多飞行器仿真的无缝切换。

## 2. 依赖关系

- **C++ 标准**: C++17 及以上
- **构建工具**: CMake 3.16 及以上
- **核心数学库**: **Eigen3** (版本 3.3+，必需)。框架的底层矩阵运算、线性代数均基于 Eigen3 封装。
- **其他**: 框架内置了轻量级 JSON 解析器，**无需**安装第三方 JSON 库或其他大型依赖，保持了极高的轻量化。

## 3. 项目整体架构

项目采用清晰的**五层架构**，自下而上依次为：

1. **数学库 (Math Library)**: 基于 Eigen3 的封装，提供四元数、旋转矩阵、数值积分（RK4）、插值、非线性方程求根等基础数学工具。
2. **算法库 (Algorithm Library)**: 针对 GNC 领域的通用算法，如 PID 控制器、状态空间模型、各种滤波器（巴特沃斯、陷波）、坐标旋转静态公式等。
3. **接口层 (Interface)**: 定义组件之间的标准契约（如 `IDynamics`, `IGuidance`, `IImuSensor` 等），并规定了标准的数据流转结构体（如 `NavState`, `GuidanceCommand`）。
4. **服务层 (Service)**: 提供跨组件的基础设施服务，如 `CoordinateService`（树形坐标变换管理）和 `CsvDataLogger`（仿真数据记录）。
5. **应用层 (Application)**: 包含核心调度引擎（`Simulator`, `SimulationBuilder` 等）和具体的业务组件实现（如 `SimpleDynamics`, `Wgs84Earth` 等）。

## 4. 核心目录结构与模块职责

```text
/workspace
├── CMakeLists.txt              # 顶层 CMake 构建脚本
├── src/
│   └── runner.cpp              # 框架的主入口点，负责解析命令行参数并启动仿真
├── framework/
│   └── include/gnc/
│       ├── common/             # 数学库、日志宏、通用类型定义
│       ├── libraries/          # 控制算法、滤波器、坐标系转换算法库
│       ├── interfaces/         # 核心抽象接口（动力学、环境、GNC算法、传感器等）
│       ├── services/           # 跨组件服务（坐标树管理等）
│       ├── core/               # 框架核心引擎（生命周期管理、注册表、仿真构建器、组件基类）
│       └── components/         # 官方内置的具体组件实现（如 Wgs84地球模型、简单动力学等）
├── user/
│   ├── components/             # 用户自定义组件存放目录（CMake 会自动扫描并注册该目录下的头文件）
│   ├── config/missions/        # 仿真配置文件（JSON 格式）存放目录
│   └── outputs/                # 仿真运行输出的 CSV 数据文件目录
├── examples/                   # 示例代码（包含最小示例、重力转弯、三自由度仿真等）
├── templates/                  # 供用户参考或复制的组件及配置模板
└── tools/                      # 辅助工具（如 python 绘图脚本、powershell 一键运行脚本）
```

## 5. 关键类与核心机制说明

### 5.1 组件基类 `ComponentBase`
[component_base.hpp](file:///workspace/framework/include/gnc/core/component_base.hpp)
所有参与仿真的模块必须继承自 `ComponentBase`，并按需实现具体的业务接口（如 `IGuidance`）。它定义了组件的生命周期：
- `configure(config)`: 解析 JSON 配置文件中的参数。
- `injectDependencies(registry)`: 从注册表中获取依赖的其他组件指针。
- `injectServices(services)`: 从服务上下文中获取跨组件服务（如坐标服务）。
- `initialize()`: 初始化状态。
- `update(dt)`: 核心执行逻辑，按设定的频率被 Simulator 调用。
- `finalize()`: 仿真结束后的清理工作。

### 5.2 仿真构建与调度引擎
- **`SimulationBuilder`**: 负责读取 JSON 配置文件，使用工厂模式反射实例化组件，并组装成完整的 Simulator 对象。
- **`Simulator`**: 维护仿真主循环（`run()` 方法），处理时间的推进（dt），并根据每个组件配置的运行频率调用其 `update(dt)` 方法。

### 5.3 数据总线与依赖注入机制
本框架**没有**采用复杂的 Pub/Sub 消息总线，而是采用 **接口查询 + 主动拉取** 模式，保证了零拷贝的高效执行：
1. **依赖注入**: 在 `injectDependencies` 阶段，组件通过 `ScopedRegistry` 获取所需组件的接口指针（如 Navigation 获取 IMU 的指针 `IImuSensor*`）。
2. **主动拉取**: 在 `update` 阶段，直接调用接口方法获取数据（如 `imu_->getImuData()`）。

### 5.4 树形坐标服务 `CoordinateService`
位于服务层，提供强大的运行时坐标树管理功能。它允许注册两个坐标系之间的旋转关系（可以是时变的 Lambda 表达式），并通过 **LCA（最近公共祖先）** 算法自动计算任意两个连通坐标系之间的转换矩阵，自带节点级缓存，极大地简化了多坐标系转换。

## 6. 项目运行方式

### 6.1 环境准备
请确保系统中已安装 `CMake`、C++17 兼容的编译器（GCC/Clang/MSVC）以及 `Eigen3` 库。

### 6.2 命令行编译与运行
可以在项目根目录下手动构建并运行默认配置：

```bash
# 1. 创建构建目录
mkdir build && cd build

# 2. 配置并编译项目 (Release 模式可获得最佳仿真性能)
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release

# 3. 运行仿真引擎 (默认读取 user/config/missions/default.json)
./bin/gnc_sim ../user/config/missions/default.json
```

*注：Windows 系统下可直接使用 `.\tools\build_and_run.ps1` 脚本一键编译运行。*

### 6.3 查看可用组件
编译完成后，可以通过以下命令查看当前所有已注册的组件（包括内置组件和用户自定义组件）：
```bash
./bin/gnc_sim --list-components
```

## 7. 新手指南：从零搭建你的仿真模型

想要搭建自己的仿真模型，你只需要编写**自定义组件（C++）**和**配置文件（JSON）**，**无需修改框架底层代码**。下面是一步步的指南。

### 第一步：理解运行图
在你的脑海中，仿真的数据流通常如下（由配置的先后顺序决定组件的执行顺序）：
1. **环境(Earth)** 提供重力、地球自转。
2. **动力学(Dynamics)** 根据受力积分计算出真实的位置、速度、姿态。
3. **传感器(IMU/GPS)** 采集真实状态，加入噪声输出。
4. **导航(Navigation)** 处理传感器数据，输出导航解。
5. **制导(Guidance)** 依据导航解计算出期望的过载/姿态指令。
6. **控制(Controller)** 依据制导指令计算出实际的打舵/推力指令。
7. 控制力反馈给动力学，进入下一个时间步循环。

### 第二步：编写你的自定义组件
假设我们要实现一个简单的“恒定推力制导”组件。
在 `user/components/` 目录下创建一个头文件（例如 `my_guidance.hpp`），CMake 编译时会自动扫描并注册 `user/components/` 目录下的所有 `.hpp` 文件。

```cpp
#pragma once
#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/gnc/i_guidance.hpp"

class MyGuidance : public gnc::core::ComponentBase, public gnc::interfaces::IGuidance {
public:
    MyGuidance() : ComponentBase("MyGuidance") {
        setExecutionFrequency(50.0); // 设置 50Hz 运行频率
    }

    // 从 JSON 配置中读取推力参数
    void configure(const gnc::core::ConfigNode& cfg) override {
        thrust_z_ = cfg["thrust_z"].asDouble(10.0); // 默认 10.0
    }

    // 核心逻辑：每步更新时计算制导指令
    void update(double dt) override {
        // 这里可以直接设置一个简单的 Z 轴恒定加速度指令
        cmd_.acceleration_cmd = {0.0, 0.0, thrust_z_};
    }

    // 实现 IGuidance 接口必须的方法
    const gnc::interfaces::GuidanceCommand& getGuidanceCommand() const override { return cmd_; }
    void setTarget(const Vector3d& t) override { /* ... */ }
    bool isActive() const override { return true; }

private:
    double thrust_z_ = 0.0;
    gnc::interfaces::GuidanceCommand cmd_;
};

// 极其重要：将组件注册到框架的工厂中，使其可以通过 JSON 字符串实例化！
GNC_REGISTER_COMPONENT(MyGuidance, gnc::interfaces::IGuidance)
```

### 第三步：编写仿真配置文件
在 `user/config/missions/` 目录下创建一个 `my_mission.json` 文件，组装你的组件。**注意组件在 JSON 列表中的顺序即为执行顺序。**

```json
{
    "simulation": {
        "dt": 0.01,
        "duration": 10.0
    },
    "components": [
        { "type": "Wgs84Earth", "name": "earth", "config": {} },
        { 
            "type": "MyGuidance", 
            "name": "guidance", 
            "config": { "thrust_z": 25.0 } 
        },
        { "type": "SimpleDynamics", "name": "dynamics", "config": {} },
        { "type": "IdealImu", "name": "imu", "config": { "frequency_hz": 100 } },
        { "type": "SimpleNavigation", "name": "nav", "config": {} }
    ]
}
```

### 第四步：编译并运行
由于我们添加了新的头文件，需要重新运行 CMake：
```bash
cd build
cmake ..
cmake --build . --config Release
./bin/gnc_sim ../user/config/missions/my_mission.json
```

仿真运行结束后，产生的数据会默认输出到 `user/outputs/` 目录下的 CSV 文件中。你可以使用项目提供的 Python 脚本（`tools/plot_results.py`）快速绘制出飞行轨迹和姿态曲线。
