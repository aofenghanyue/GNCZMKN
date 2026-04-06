# 新手全流程教程：基于 CAV-H 的 3DOF 仿真

本教程是整个 Wiki 最核心的实战篇。我们将以项目中自带的 `examples/03_cavh_3dof` 为例，**事无巨细地带您从零开始，使用 GNC 框架搭建一个高超声速飞行器 (CAV-H) 的三自由度 (3DOF) 下滑仿真模型。**

## 1. 案例背景与任务目标

**CAV-H** 是一种经典的高超声速滑翔飞行器。我们的任务目标是：
1.  **环境**：地球采用标准的 WGS84 球体模型，大气采用标准大气模型。
2.  **物理模型**：质量设定为 900kg（恒定），气动系数（升力、阻力）通过公式计算（与马赫数和攻角相关）。
3.  **动力学**：采用 3 自由度球坐标系质点运动学积分器（`Dynamics3DOF_SphericalEarth`）。
4.  **制导策略**：根据高度剖面，分段插值给定期望的攻角（Programmed AoA，程序攻角），同时保持倾侧角（Sigma）为 0 度。
5.  **仿真终止条件**：当飞行器高度下降到 10km（10000m）时，自动停止仿真。

---

## 2. 第一步：编写 C++ 自定义组件

根据上述任务，我们需要自己手写三个组件：**质量组件**、**气动组件**和**制导组件**。这三个文件均放置在 `examples/03_cavh_3dof/` 下。

### 2.1 编写质量组件 (`cavh_mass.hpp`)
质量组件非常简单，只需继承 `IMassProperty` 接口并返回 900kg 即可。为了让日志能记录质量，我们还继承了 `IObservable`。

```cpp
#pragma once
#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/vehicle/i_mass_property.hpp"
#include "gnc/interfaces/infrastructure/i_observable.hpp"

class CavhMass : public gnc::core::ComponentBase, 
                 public gnc::interfaces::IMassProperty, 
                 public gnc::interfaces::IObservable {
public:
    CavhMass() : ComponentBase("CavhMass") {} // 构造函数，设定名字
    
    // 1. 从 JSON 配置文件读取质量
    void configure(const gnc::core::ConfigNode& config) override {
        mass_ = config["mass_kg"].asDouble(900.0);
    }
    
    // 2. 实现接口要求的方法
    double getMass() const override { return mass_; }
    gnc::Matrix3d getInertiaMatrix() const override { return gnc::Matrix3d::Identity(); }
    gnc::Vector3d getCenterOfMass() const override { return gnc::Vector3d::Zero(); }
    void updateMassProperties(double) override {}
    void update(double) override {} // 质量不变，不需要每步更新
    
    // 3. 暴露给 CSV 记录器
    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("mass", [this]() { return mass_; });
        return builder.build();
    }
private:
    double mass_ = 900.0;
};
// 4. 宏注册：极其重要！
GNC_REGISTER_COMPONENT(CavhMass, gnc::interfaces::IMassProperty)
```

### 2.2 编写气动组件 (`cavh_aerodynamics.hpp`)
气动组件负责提供升力系数(CL)和阻力系数(CD)。计算 CL 和 CD 需要知道当前的**攻角(Alpha)**和**马赫数(Mach)**。
因此，气动组件必须向框架“索要”制导组件（获取攻角指令）和动力学组件（获取速度计算马赫数）的指针！

```cpp
// ... 省略头文件包含 ...
class CavhAerodynamics : public gnc::core::ComponentBase,
                         public gnc::interfaces::IAeroCoefficients,
                         public gnc::interfaces::IObservable {
public:
    CavhAerodynamics() : ComponentBase("CavhAerodynamics") {}

    // 1. 读取气动公式的常数项
    void configure(const gnc::core::ConfigNode& config) override {
        cl_alpha_ = config["cl_alpha_per_rad"].asDouble(1.6);
        cd0_ = config["cd0"].asDouble(0.08);
        reference_area_ = config["reference_area"].asDouble(0.48);
        // ... (其他参数省略)
    }

    // 2. 依赖注入：获取制导和动力学的指针！
    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        guidance_ = registry.getByName<gnc::interfaces::IGuidance3DOF>("guidance");
        dynamics_ = registry.getByName<gnc::interfaces::IDynamicsModel>("dynamics");
    }

    // 3. 计算气动系数的核心逻辑
    gnc::interfaces::AeroCoefficients computeCoefficients(double alpha, double, double mach) const override {
        // 使用简单的多项式拟合高超气动特性
        double cl = 0.0 + cl_alpha_ * alpha;
        double cd = cd0_ + 1.25 * alpha * alpha + 0.015 * std::max(0.0, mach - 5.0);
        return {cl, cd, 0.0, 0.0, 0.0, 0.0};
    }

    // 4. 提供给动力学拉取的接口方法（这部分比较特殊，动力学通常拉取 currentCoefficients）
    // 为了演示，我们在内部私有方法中获取最新的 alpha 和 mach：
private:
    gnc::interfaces::AeroCoefficients currentCoefficients() const {
        // 从制导拉取期望攻角 (假设 3DOF 能够完美跟踪攻角)
        const double alpha = guidance_ ? guidance_->getFlightCommand().alpha : 0.0;
        // 从动力学拉取速度，除以声速得到马赫数 (简化的声速常数)
        const double mach = dynamics_ ? dynamics_->getStateValue("velocity") / 340.0 : 0.0;
        return computeCoefficients(alpha, 0.0, mach);
    }
    
    // ... 成员变量和注册宏省略 ...
```

### 2.3 编写程序攻角制导组件 (`cavh_programmed_aoa.hpp`)
根据飞行高度，插值输出攻角。它需要向框架“索要”动力学组件的指针，以获取当前高度。

```cpp
class CavhProgrammedAoA : public gnc::core::ComponentBase, public gnc::interfaces::IGuidance3DOF {
public:
    CavhProgrammedAoA() : ComponentBase("CavhProgrammedAoA") {
        setExecutionFrequency(20.0); // 制导频率 20Hz
    }
    
    // 依赖注入：索要动力学指针
    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        dynamics_ = registry.getByName<gnc::interfaces::IDynamicsModel>("dynamics");
    }
    
    // 核心更新：根据高度计算攻角
    void update(double) override {
        // 1. 拉取高度
        double altitude = dynamics_ ? dynamics_->getStateValue("altitude") : 60000.0;
        // 2. 根据高度数组插值得到攻角 (插值函数逻辑略)
        command_.alpha = interpolateAlpha(altitude) * 3.14159 / 180.0; // 转弧度
        command_.sigma = 0.0; // 倾侧角保持为 0
    }
    // ... 暴露指令及注册宏省略 ...
```

---

## 3. 第二步：编写 JSON 仿真配置文件

代码写好了，如何将它们拼装起来运行呢？在 `examples/03_cavh_3dof/` 下创建 `cavh_mission.json`。
**注意组件数组中的顺序！必须是：环境 -> 质量 -> 气动 -> 制导 -> 动力学！**（动力学最后执行，因为它是积分器，依赖前面算出的所有力和系数）。

```json
{
    "simulation": {
        "dt": 0.1,             // 仿真步长 0.1秒 (10Hz)
        "duration": 600.0,     // 最大仿真时长 600秒
        "integrator": "rk4",   // 采用四阶龙格库塔积分器
        "stop_conditions": [
            {
                "type": "component_field_below",
                "component": "dynamics",
                "field": "altitude",
                "value": 10000.0,  // 高度低于 10000m 时自动停止
                "description": "Altitude below 10km"
            }
        ]
    },
    "outputs": {
        "directory": "user/outputs/{timestamp}", // 数据输出目录
        "format": "csv",
        "session_name": "cavh_3dof",
        "record": {
            "dynamics": ["altitude", "velocity", "mach", "alpha_deg"],
            "aero": ["CL", "CD", "lift_to_drag"],
            "guidance": "all" // 记录制导组件所有暴露的变量
        }
    },
    "components": [
        { "type": "StandardAtmosphere", "name": "atmosphere", "config": {} },
        { "type": "SphericalGravity", "name": "gravity", "config": {} },
        { 
            "type": "CavhMass", 
            "name": "mass", 
            "config": { "mass_kg": 900.0 } 
        },
        { 
            "type": "CavhAerodynamics", 
            "name": "aero", 
            "config": { "cl_alpha_per_rad": 1.8 } // 覆盖 C++ 中的默认配置
        },
        { 
            "type": "CavhProgrammedAoA", 
            "name": "guidance", 
            "config": {
                "schedule_altitude_m": [60000, 45000, 30000, 15000],
                "schedule_alpha_deg": [20, 15, 10, 6]
            }
        },
        { 
            "type": "Dynamics3DOF_SphericalEarth", 
            "name": "dynamics", 
            "config": {
                "initial_state": { // 给定高超滑翔的初始状态！
                    "longitude": 110.0,
                    "latitude": 30.0,
                    "altitude": 60000.0,   // 起始高度 60km
                    "velocity": 3200.0,    // 初始速度 3.2km/s
                    "flight_path_angle": -6.0, // 初始弹道倾角
                    "heading_angle": 90.0
                }
            }
        }
    ]
}
```

---

## 4. 第三步：编写 `main.cpp` 并编译运行

### 4.1 `main.cpp`
为了让框架知道我们写了自定义组件，我们需要在 `main.cpp` 中 `#include` 它们（利用 C++ 的静态全局变量初始化机制，包含头文件即可触发宏 `GNC_REGISTER_COMPONENT` 自动向工厂注册！）。

```cpp
#include "gnc/common/logger.hpp"
#include "gnc/components/_builtin_register.hpp" // 注册框架内置组件
#include "gnc/core/simulation_builder.hpp"

// 必须包含我们写的三个自定义组件的头文件，触发宏注册！
#include "cavh_aerodynamics.hpp"
#include "cavh_mass.hpp"
#include "cavh_programmed_aoa.hpp"

int main() {
    gnc::core::SimulationBuilder builder;
    // 加载我们刚写的 JSON 配置文件
    if (!builder.loadConfig("examples/03_cavh_3dof/cavh_mission.json")) return 1;

    // 一键构建仿真器并运行！
    auto& simulator = builder.build();
    simulator.run();
    
    return 0;
}
```

### 4.2 编译与运行
由于我们在 `examples` 下添加了代码，如果项目顶层 `CMakeLists.txt` 包含了它，我们可以直接在根目录下构建：

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release

# 运行生成的执行文件
./bin/cavh_3dof_example
```

> **恭喜您！**
> 运行结束后，您可以在 `user/outputs/` 的最新时间戳文件夹下找到 `cavh_3dof.csv` 文件。使用 Python 脚本绘制出 `altitude` 和 `velocity` 的曲线，您就能看到完美的 CAV-H 典型高超声速滑翔弹道了！
