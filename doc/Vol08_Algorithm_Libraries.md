# Volume 8: 控制与滤波算法库 (Algorithm Libraries)

## 1. 增强型 PID 控制器 (`pid_controller.hpp`)
在 `gnc::libraries::PidController` 中，我们提供了一个工业级的 PID 实现。它不仅包含了基本的 P, I, D，还解决了实际工程中的常见痛点。

### 1.1 抗积分饱和 (Anti-Windup)
执行机构（如舵机）总是有物理偏角限制的。当误差长期存在且舵机满偏时，传统的积分器会不断累积，导致“积分饱和”。当误差反转时，舵机需要很长时间才能退饱和，引发剧烈超调。
框架提供了两种解决策略：
- **`Clamp`（钳位）**：简单粗暴地限制积分项（`integral_min` 到 `integral_max`）。
- **`BackCalculation`（回算法）**：工业界推荐。利用未限幅输出与限幅后输出的差值，乘以回算增益 `kb`，反向补偿积分项。当执行器饱和时，积分器会自动停止累加甚至回退。

### 1.2 微分滤波 (Derivative Filter)
现实传感器的量测总是带有噪声。如果直接计算 $d_{err}/dt$，噪声会被极度放大，导致执行机构疯狂抖动。
```cpp
pid.setDerivativeFilter(0.05); // 设置时间常数 tau = 0.05s
```
这会在微分项上串联一个一阶低通滤波器 $\frac{s}{1 + \tau s}$，使得高频噪声被有效衰减。

## 2. 数字滤波器 (`filters/`)

### 2.1 巴特沃斯滤波器 (`ButterworthFilter`)
基于 Biquad（双二阶节）级联实现的无限脉冲响应（IIR）数字滤波器。
- **特性**：具有最大平坦的通带幅频响应，没有纹波。
- **用法**：
  ```cpp
  // 创建一个 4 阶，截止频率 10Hz，采样频率 100Hz 的低通滤波器
  auto lpf = ButterworthFilter::lowpass(4, 10.0, 100.0);
  double filtered_value = lpf.filter(noisy_sensor_value);
  ```
- **内部实现**：通过 Bilinear Transform（双线性变换）将 s 域的极点映射到 z 域，并分解为多个 `Biquad` 结构，保证了高阶滤波器的数值稳定性。

### 2.2 简单一阶低通 (`FirstOrderLowpass`)
对于不需要陡峭截止特性的简单场景，使用简单的 RC 滤波器：
$y_k = \alpha x_k + (1-\alpha) y_{k-1}$

## 3. 状态空间模型 (`state_space.hpp`)
对于 MIMO（多输入多输出）或现代控制理论（LQR / H_infinity），框架提供了 `StateSpaceModel`。
- 支持连续系统 $\dot{x} = Ax + Bu$ 和离散系统 $x_{k+1} = A_d x_k + B_d u_k$。
- 内置了极点（特征值）计算与稳定性判定 (`isStable()`)。
- 对于连续系统，调用 `update()` 默认使用 RK4 积分进行单步离散化。

## 4. 坐标系旋转公式 (`rotations.hpp`)
该头文件提供了纯粹的静态数学公式，不带有任何类状态。
- 涵盖了航天常用坐标系：`ECEF`, `NED`, `ENU`, `NSE`, `LAUNCH`, `ECI`。
- 提供气动专用坐标系变换：`Body` $\leftrightarrow$ `Wind` $\leftrightarrow$ `Stability`。
- 所有的旋转矩阵约定为左乘列向量：$v_{new} = R \times v_{old}$。