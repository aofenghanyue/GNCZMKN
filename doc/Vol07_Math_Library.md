# Volume 7: 高级数学库 (Math Library)

## 1. 纯数学工具：`gnc::math`
位于 `gnc/common/math/` 下的工具集是纯函数或纯数学类，**完全没有框架依赖**，不了解组件系统。

## 2. 数值微积分 (`calculus.hpp`)
提供了从 ODE 求解到梯度的丰富支持。
- **定步长 RK4 积分 (`rk4_step`)**：经典的 4 阶龙格库塔算法，对性能最友好。
- **自适应 RK45 积分 (`rk45_step`, `integrate_ode`)**：实现了 Dormand-Prince 算法。带有误差估计和自动步长缩放（Safety Factor）。这通常用于线下轨迹优化或不依赖固定实时步长的离线计算。
- **数值求导 (`numerical_gradient`, `numerical_jacobian`, `numerical_hessian`)**：
  - 基于中心差分算法，默认使用 $\sqrt{\epsilon}$ 自动确定差分步长。
  - 这使得用户不需要手写复杂的解析雅可比矩阵即可快速验证扩展卡尔曼滤波 (EKF) 或优化算法。

## 3. 多维插值与查表 (`interp.hpp`)
气动数据的核心工具。所有类在超出范围时都支持三种外推策略：`Clamp`（钳位）、`Linear`（线性）和 `Error`（抛出异常）。
- **`LookupTable1D / 2D / 3D`**：
  - 支持非均匀网格。
  - 内部使用 `std::upper_bound` 进行二分查找，寻找索引 $O(\log N)$。
  - 支持双线性、三线性插值。
- **`CubicSpline` (三次样条)**：
  - 保证一阶、二阶导数连续。对于平滑数据（如轨道规划）非常有效。
- **`AkimaSpline` (Akima 插值)**：
  - **航空航天领域极其重要的特性！** 传统的 Cubic Spline 在阶跃数据（例如超音速激波区域的气动数据）处会产生巨大的“过冲（Overshoot）”。Akima 插值是一种局部插值法，能够完美避免过冲，特别适合插值真实的激波风洞数据。

## 4. 四元数与旋转算子 (`quaternion.hpp`, `rotation.hpp`)

### 4.1 四元数约定
框架严格采用了航天领域的约定：
- **存储顺序**：`w, x, y, z`，实部 `w` 在前。
- **组合法则**：采用**右乘法则**。即 $q_{total} = q_1 \otimes q_2$ 表示“先执行 $q_1$ 旋转，再执行 $q_2$ 旋转”。这与对应的方向余弦矩阵（DCM）完全同构：$M(q_1 \otimes q_2) = M(q_2) M(q_1)$。
- **旋转公式**：采用共轭三明治公式 $v' = q^* \otimes v \otimes q$。

### 4.2 统一的 `Rotation` 类
为了避免混淆，`Rotation` 类对四元数进行了封装，作为一个抽象的旋转算子。
```cpp
Rotation R1 = Rotation::fromEuler(roll, pitch, yaw, EulerSeq::ZYX);
Rotation R2 = Rotation::rotateX(M_PI / 2);
Rotation R_total = R1 * R2;
Vector3 v_rotated = R_total.apply(v_body);
```
它支持从 12 种不同的欧拉角序列（如 ZYX, ZYZ, XYZ 等）转换为四元数，并能随时导出为 DCM 矩阵。