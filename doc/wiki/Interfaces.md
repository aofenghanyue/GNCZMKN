# 接口设计与核心组件规范

本框架的所有组件都必须遵循一套**标准的接口契约**。GNC 仿真环境由无数个“即插即用”的积木（组件）拼装而成，而这些积木之间的接头，就是位于 `framework/include/gnc/interfaces/` 目录下的接口定义。

## 1. 接口的设计理念

在 GNC 框架中，所有跨模块的交互都必须通过**纯虚接口（Abstract Interface）**进行。
- **无实现细节**：接口类通常只包含纯虚函数（如 `virtual double getMass() const = 0;`）。
- **零拷贝拉取**：接口方法倾向于返回基本类型、`const` 引用或简单的结构体，以避免深拷贝。
- **强制依赖声明**：组件在生命周期初始化时，必须明确它需要哪些接口指针。

---

## 2. 核心接口速查表

以下列出框架内最核心的几类接口及其主要能力：

### 2.1 环境接口 (Environment)
这类接口通常是全局共享的，负责提供不随单一飞行器状态改变的自然环境数据。
*   `IAtmosphereModel`: 提供基于高度的**大气密度、压力、温度和声速**（`getDensity`, `getSpeedOfSound`）。
*   `IGravityModel`: 提供基于位置的**重力加速度**向量（`getGravity`）。
*   `IEarthModel`: 提供**WGS84 坐标系转换**（LLA 到 ECEF 的互相转换）、地球半径和自转角速度。

### 2.2 飞行器物理特性接口 (Vehicle Properties)
描述飞行器自身的固有物理属性，是动力学积分器的重要输入。
*   `IMassProperty`: 提供**质量**（`getMass()`）、**惯性张量**（`getInertiaMatrix()`）以及**质心位置**。
*   `IAeroCoefficients`: 根据马赫数、攻角、侧滑角等提供**气动力系数**（如 `CL`, `CD`, `CY`, `Cl`, `Cm`, `Cn`）及参考面积/长度。

### 2.3 GNC 算法接口 (Guidance, Navigation, Control)
框架中最常被用户自定义的业务层接口。
*   `IGuidance3DOF`: 三自由度制导接口，主要输出为**期望攻角（$\alpha$）、期望倾侧角（$\sigma$）**，即 `FlightCommand3DOF`。
*   `IGuidance6DOF`: 六自由度制导接口，输出为**期望的三轴加速度和姿态指令**（`GuidanceCommand`）。
*   `INavigation`: 导航滤波器接口，输出为当前估算的**导航状态**（包含位置、速度、姿态四元数的 `NavState` 结构体）。
*   `IController`: 控制器接口，输入制导指令和导航状态，输出具体的**执行器指令**（如舵面偏角指令、推力指令的 `ActuatorCommand`）。

### 2.4 动力学模型接口 (Dynamics)
*   `IDynamicsModel`: 仿真的心脏。它负责维护和积分飞行器的真实状态（Truth State）。
    *   提供获取状态值的通用方法：`getStateValue(std::string_view name)`（如获取 "altitude", "mach", "velocity"）。
    *   提供施加外力/力矩的接口：`addForce`, `addTorque`。

### 2.5 基础设施与日志接口 (Infrastructure)
*   `IObservable`: **至关重要的日志接口**。任何希望自己的内部变量被 CSV 记录器导出的组件，都必须继承并实现此接口的 `getObservableFields()` 方法。

---

## 3. 跨模块的数据结构 (Data Types)

为了避免参数列表过长，框架在 `data_types.hpp` 中定义了标准的数据载体（POD 结构体）：

*   **`ImuData`**: 包含 `acceleration` (三轴比力), `angular_velocity` (三轴角速度), `timestamp`。
*   **`NavState`**: 包含 `position` (ECEF系), `velocity` (ECEF系), `attitude` (NED到BODY的四元数), `angular_velocity` (BODY系), `timestamp`。
*   **`FlightCommand3DOF`**: 三自由度指令，包含 `alpha` (攻角, rad), `sigma` (倾侧角, rad), `timestamp`。
*   **`AeroCoefficients`**: 气动系数结构体，包含 `CL, CD, CY`（升力、阻力、侧力系数）和 `Cl, Cm, Cn`（滚转、俯仰、偏航力矩系数）。

> 下一步：想知道这些接口如何组合成一个实际可运行的组件，请参阅 [**自定义组件开发指南**](Component_Development.md)。
