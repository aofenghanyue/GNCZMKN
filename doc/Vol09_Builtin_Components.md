# Volume 9: 内置组件参考 (Built-in Components)

框架内置了多个开箱即用的环境模型、传感器和动力学模型，主要位于 `gnc/components/` 目录下。

## 1. 环境组件 (Environment)
这些组件通常在 JSON 的 `"environment"` 节点下注册，作为全局服务提供给所有飞行器。

### 1.1 WGS84 地球模型 (`Wgs84Earth`)
- **功能**：实现了 `IEarthModel` 接口。提供地球长半轴 $a = 6378137.0$ m 和自转角速度。
- **限制**：当前的 `geodeticToEcef` 和 `ecefToGeodetic` 简化了扁率处理（视为球体近似）。如需高精度弹道计算，您可以继承该类重写转换函数。

### 1.2 标准大气模型 (`StandardAtmosphere`)
- **功能**：实现了 `IAtmosphereModel`。这是 1976 年美国标准大气模型（USSA1976）的 C++ 实现。
- **特性**：
  - 支持 $0 \sim 86$ km 高度范围。
  - 分为 7 层，交替使用等温层（指数衰减）和温度梯度层（多项式衰减）。
  - 在初始化时（`initLayers`）预计算了每层底部的压力 `p_base`，运行时只需一次 O(1) 的层查找（`findLayer`）和少量浮点运算，极大地提高了密度、温度和音速（Mach 数计算基础）的查询速度。

### 1.3 球形重力模型 (`SphericalGravity`)
- **功能**：实现了 `IGravityModel` 接口。基于 $g = g_0 \frac{R_e^2}{r^2}$ 计算万有引力矢量，不包含 J2 摄动。

## 2. 动力学组件 (Dynamics)

### 2.1 球地三自由度质点动力学 (`Dynamics3DOF_SphericalEarth`)
- **功能**：这是导弹、火箭等弹道设计中最核心的 3DOF 运动方程。
- **状态空间**：使用 `StateLayout` 管理了 6 个连续状态变量：
  - 经度 `longitude`
  - 纬度 `latitude`
  - 高度 `altitude`
  - 速度 `velocity`
  - 弹道倾角 `flight_path_angle` ($\gamma$)
  - 航向角 `heading_angle` ($\psi$)
- **方程实现**：在 `computeDerivatives` 中，它主动拉取了 `IAtmosphereModel`、`IGravityModel`、`IAeroCoefficients`、`IMassProperty` 和 `IGuidance3DOF`（如果存在），计算出气动力（Lift/Drag）并推演出状态的微分。

### 2.2 简单六自由度刚体动力学 (`SimpleDynamics`)
- **功能**：实现了 `IDynamicsModel`，但内部采用了简单的牛顿-欧拉方程。
- **状态空间**：13 维（位置，速度，姿态四元数，角速度）。
- **外部力矩**：通过 `addForce` 和 `addTorque` 被动接受控制器的推力/力矩累加。

## 3. 传感器组件 (Sensors)

### 3.1 理想 IMU (`IdealImu`)
- **功能**：实现了 `IImuSensor` 接口。它不产生任何高斯白噪声或零偏。
- **工作机制**：依赖 `IDynamicsModel`。在 `update(dt)` 时，直接从真实动力学中提取加速度和角速度，赋值给 `ImuData`。

## 4. 导航解算 (Navigation)

### 3.2 简单导航 (`SimpleNavigation`)
- **功能**：实现了 `INavigation` 接口。
- **工作机制**：在 `update(dt)` 时拉取 `IImuSensor` 的数据，对加速度和角速度进行简单的欧拉积分。这用于演示“真实状态 $\rightarrow$ IMU $\rightarrow$ 导航状态”的完整 GNC 闭环信息流。