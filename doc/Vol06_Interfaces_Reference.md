# Volume 6: 接口契约参考 (Interfaces Reference)

## 1. 契约层的作用
在 `gnc/interfaces/` 目录下定义了所有的虚拟基类。这些基类没有任何成员变量，仅包含纯虚函数。它们是框架的“骨架”。通过依赖倒置原则，组件间只依赖接口而不依赖具体实现。

## 2. 环境模型接口 (Environment)
- **`IEarthModel` (地球模型)**
  - `getRadius()`：获取地球半径。
  - `getRotationRate()`：获取地球自转角速度。
  - `geodeticToEcef() / ecefToGeodetic()`：提供 LLA 与 ECEF 坐标系的转换。
- **`IGravityModel` (引力模型)**
  - `getGravity(const Vector3d& position)`：根据空间位置获取引力矢量（考虑 J2 项或球形假设）。
- **`IAtmosphereModel` (大气模型)**
  - `getDensity()`, `getPressure()`, `getTemperature()`, `getSpeedOfSound()`：根据高度获取大气参数。这对于气动力计算至关重要。

## 3. 动力学与运动学接口 (Dynamics)
- **`IDynamicsModel`**
  - 这是框架中最复杂的接口。为了配合数值积分器，必须提供：
  - `getStateLayout()`：返回动态状态向量的布局定义。
  - `getState() / setState()`：供积分器提取和覆盖状态。
  - `computeDerivatives(t, x, dxdt)`：最核心的方程，计算当前状态下的导数 $\dot{x}$。

## 4. 制导、导航与控制接口 (GNC)
为了适配不同的自由度，框架分离了 3DOF 和 6DOF 的制导接口。

### 4.1 3DOF 制导 (`IGuidance3DOF`)
- `getFlightCommand()`：返回 `FlightCommand3DOF`。
  - `alpha`：攻角 (rad)
  - `sigma`：倾侧角 (rad)

### 4.2 6DOF 制导 (`IGuidance6DOF`)
- `getGuidanceCommand()`：返回 `GuidanceCommand6DOF`。
  - `acceleration_cmd`：三轴加速度指令矢量。
  - `attitude_cmd`：姿态指令矢量。

### 4.3 质量与气动接口 (Vehicle Properties)
- **`IMassProperty`**
  - `getMass()`：获取当前质量。可用于实现燃料消耗动态。
- **`IAeroCoefficients`**
  - `computeCoefficients(alpha, beta, mach)`：根据飞行状态返回 `AeroCoefficients` (包含 $C_L, C_D, C_Y, C_l, C_m, C_n$)。
  - `getReferenceArea() / getReferenceLength()`：返回参考面积和特征长度。