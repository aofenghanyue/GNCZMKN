# 08 代码地图与模块参考

本文件不是逐行解释源码，而是给出一张“当前仓库有效代码地图”，帮助用户知道每个模块的职责边界、可复用方式以及在用户手册中的定位。

## 8.1 入口、构建与用户工作区

| 路径 | 作用 | 用户需要知道什么 |
| --- | --- | --- |
| `CMakeLists.txt` | 顶层构建入口 | 扫描当前自定义组件根，生成自动注册头文件，构建 `gnc_sim` 和测试 |
| `framework/CMakeLists.txt` | 框架库定义 | 框架本体是 INTERFACE 头文件库，主要依赖 Eigen3 |
| `src/runner.cpp` | 默认运行入口 | 支持 `--config`、向上搜索默认任务路径、列组件，并按 starter/custom 分组显示类型 |
| `tools/build_and_run.ps1` | 一键构建运行脚本 | 适合 Windows 下快速验证、列组件和运行任务 |
| `tools/plot_results.py` | 结果后处理脚本 | 面向 CSV 结果做简单绘图和文本摘要 |
| `README.md` | 仓库入口说明 | 首次进入仓库时优先看，用于建立“基座 / 起步件 / 样板工程 / 用户工程区”的基本心智模型 |
| `templates/guidance_template.hpp` | 制导组件模板 | 自定义制导的起点，默认先用 `bind(...)` 起步 |
| `templates/controller_template.hpp` | 控制组件模板 | 自定义控制组件的起点，默认用 `guidance` 必需 + `nav` 可选 |
| `templates/navigation_template.hpp` | 导航组件模板 | 自定义导航组件的起点，默认先用 `bind(...)` 起步 |
| `templates/mission_template.json` | mission 模板 | 新任务文件起点，使用 `_comment` 提示配置意图 |
| `user/README.md` | 用户工程区说明 | 说明 `user/active_project`、`user/<project>/components/` 和 `user/<project>/config/mission.json` 约定 |
| `user/components/README.md` | legacy 组件目录说明 | 说明旧的 `user/components/` 已降级为过渡目录 |
| `user/example_01_minimal/components/constant_guidance.hpp` | 当前最小工程组件示例 | 最小可运行制导组件，也是自动注册链路的真实例子 |
| `user/example_03_cavh_3dof/config/mission.json` | 当前默认首跑任务 | 默认入口读取的闭环任务，直接运行即可得到 CSV 与摘要 |
| `user/example_01_minimal/config/mission.json` | 用户最小任务 | 当前仓库可直接运行的最小闭环任务 |

## 8.2 示例与测试

| 路径 | 作用 | 用户需要知道什么 |
| --- | --- | --- |
| `user/example_02_gravity_turn/simple_gravity_turn.cpp` | 自包含专用程序示例 | 演示 `loadConfigString()`、嵌入式使用和自动数据记录 |
| `user/example_03_cavh_3dof/components/cavh_programmed_aoa.hpp` | 3DOF 制导示例 | 按高度程序输出攻角/滚转指令 |
| `user/example_03_cavh_3dof/components/cavh_aerodynamics.hpp` | 气动模型示例 | 根据攻角和马赫数提供气动系数，并暴露可观测量 |
| `user/example_03_cavh_3dof/components/cavh_mass.hpp` | 质量模型示例 | 提供质量属性并支持数据记录 |
| `user/example_03_cavh_3dof/config/mission.json` | 完整 3DOF mission | 展示较完整的工程化装配方式 |
| `tests/test_cross_entity_access.cpp` | 作用域访问测试 | 说明跨实体全名访问是受支持的 |
| `tests/test_starter_dependency_preflight.cpp` | starter 依赖预检查测试 | 说明轻量 starter 组件即使不写 `IDependencyDeclarer` 也会在构建期被拦下 |
| `tests/test_declared_dependency_preflight.cpp` | declarer 绑定漂移测试 | 说明 declarer 组件也会继续预检查真实 `injectDependencies()` 绑定 |
| `tests/test_integrators.cpp` | 积分器测试 | 验证 RK4 和 Euler 的基本精度关系 |
| `tests/test_math_headers.cpp` | 数学头编译测试 | 说明数学库可直接复用 |
| `tests/test_transform.cpp` | 坐标/旋转数值验证 | 说明数学与坐标库经过 MATLAB 对照验证 |

## 8.3 `common/` 基础层

| 路径 | 作用 | 用户需要知道什么 |
| --- | --- | --- |
| `framework/include/gnc/common/logger.hpp` | 轻量日志系统 | 框架和用户组件都可用 `LOG_INFO` 等宏输出诊断 |
| `framework/include/gnc/common/types.hpp` | 基础标识类型 | 定义仿真时间、组件标识等基础概念 |
| `framework/include/gnc/common/math_types.hpp` | 轻量数学类型 | 定义简版 `Vector3d`、`Quaterniond`、`Matrix3d`，常用于接口数据包 |
| `framework/include/gnc/common/math/math.hpp` | 数学总入口 | 统一导出旋转、变换、插值、优化、统计等能力 |
| `framework/include/gnc/common/math/eigen_types.hpp` | Eigen 类型与常量 | 定义 `VectorX`、`MatrixX`、角度换算、反对称矩阵等 |
| `framework/include/gnc/common/math/quaternion.hpp` | 四元数类 | 姿态表示、旋转矩阵转换、角速度积分 |
| `framework/include/gnc/common/math/rotation.hpp` | 统一旋转算子 | 用统一接口包装四元数、欧拉角和 DCM 变换 |
| `framework/include/gnc/common/math/transform.hpp` | 刚体变换类 | 旋转加平移的齐次变换封装 |
| `framework/include/gnc/common/math/euler_sequences.hpp` | 欧拉序列工具 | 12 种欧拉角序列与矩阵/四元数转换 |
| `framework/include/gnc/common/math/calculus.hpp` | 数值积分与微分 | RK4/RK45、数值梯度、Jacobian、Hessian、微分近似 |
| `framework/include/gnc/common/math/interp.hpp` | 插值与查表 | 1D/2D/3D 查表、样条、最小二乘拟合 |
| `framework/include/gnc/common/math/linalg.hpp` | 线性代数扩展 | SVD、广义逆、Cholesky、秩、条件数等 |
| `framework/include/gnc/common/math/nonlinear.hpp` | 常见非线性环节 | 速率限制、回差、继电、量化、摩擦等 |
| `framework/include/gnc/common/math/optimization.hpp` | 优化工具 | 黄金分割、Brent、Nelder-Mead、梯度下降、简易 QP |
| `framework/include/gnc/common/math/roots.hpp` | 求根工具 | Newton、二分、Brent、多项式根 |
| `framework/include/gnc/common/math/special_functions.hpp` | 特殊函数 | 饱和、死区、误差函数、Bessel 函数等 |
| `framework/include/gnc/common/math/statistics.hpp` | 统计工具 | 高斯分布、随机数、协方差传播、样本统计 |

## 8.4 `core/` 框架核心层

| 路径 | 作用 | 用户需要知道什么 |
| --- | --- | --- |
| `framework/include/gnc/core/component_base.hpp` | 组件基类 | 定义生命周期、频率、仿真时间接口，是所有组件扩展的起点 |
| `framework/include/gnc/core/component_factory.hpp` | 组件工厂 | 管理“类型名 -> 组件创建器”，并记录 starter/custom 类别与注册来源 |
| `framework/include/gnc/core/component_registry.hpp` | 组件注册表 | 管理实例名、接口映射、注册顺序，是依赖注入与调度基础 |
| `framework/include/gnc/core/config_manager.hpp` | 配置系统 | 轻量 JSON 解析、节点访问、未使用字段检查 |
| `framework/include/gnc/core/simulation_builder.hpp` | 仿真构建器 | 从 mission 生成完整仿真，包括组件、服务、停止条件和输出 |
| `framework/include/gnc/core/simulator.hpp` | 仿真器 | 管理主循环、频率调度、积分器调用、回调和结束流程 |
| `framework/include/gnc/core/scoped_registry.hpp` | 作用域视图 | 支持同实体短名查找和跨实体全名查找 |
| `framework/include/gnc/core/service_context.hpp` | 服务容器 | 管理按类型注册的共享服务 |
| `framework/include/gnc/core/dependency_validator.hpp` | 依赖校验 | 检查组件声明的依赖是否被满足，并管理执行阶段 |
| `framework/include/gnc/core/auto_data_logger.hpp` | 自动数据记录 | 根据 `outputs` 配置自动发现 `IObservable` 字段并写 CSV |
| `framework/include/gnc/core/csv_record_sink.hpp` | CSV 输出后端 | 自动记录系统的 CSV 实现 |
| `framework/include/gnc/core/simulation_summary.hpp` | 仿真摘要生成器 | 在输出目录写 `summary.txt`，并记录组件类型、starter/custom 类别与来源 |
| `framework/include/gnc/core/observable_helpers.hpp` | 可观测字段辅助类 | 帮助组件快速组织标量/向量/四元数字段 |
| `framework/include/gnc/core/state_layout.hpp` | 状态布局 | 管理 `IDynamicsModel` 中状态名与索引映射 |
| `framework/include/gnc/core/string_utils.hpp` | 字符串工具 | 提供编辑距离和相近名字建议 |
| `framework/include/gnc/core/data_logger.hpp` | 旧数据记录器 | 兼容旧设计，当前不再是推荐主路径 |
| `framework/include/gnc/core/data_formatters.hpp` | 旧格式化辅助 | 为旧记录系统提供常见数据结构格式化 |
| `framework/include/gnc/core/integrators/euler_integrator.hpp` | Euler 积分器 | 简单快速，精度较低 |
| `framework/include/gnc/core/integrators/rk4_integrator.hpp` | RK4 积分器 | 当前默认积分器 |

## 8.5 `interfaces/` 抽象接口层

| 路径 | 作用 | 用户需要知道什么 |
| --- | --- | --- |
| `framework/include/gnc/interfaces/data_types.hpp` | 标准数据包 | IMU、GPS、执行器、扰动等跨组件数据结构 |
| `framework/include/gnc/interfaces/dynamics/i_dynamics_model.hpp` | 动力学接口 | 任何状态推进模型的核心接口 |
| `framework/include/gnc/interfaces/environment/i_atmosphere_model.hpp` | 大气接口 | 提供密度、压强、温度、声速 |
| `framework/include/gnc/interfaces/environment/i_earth_model.hpp` | 地球接口 | 提供半径、自转和坐标转换 |
| `framework/include/gnc/interfaces/environment/i_gravity_model.hpp` | 重力接口 | 提供重力向量和海平面重力 |
| `framework/include/gnc/interfaces/gnc/i_guidance.hpp` | 6DOF 制导别名接口 | 为 `IGuidance6DOF` 提供兼容命名 |
| `framework/include/gnc/interfaces/gnc/i_guidance_6dof.hpp` | 6DOF 制导接口 | 输出加速度/姿态指令 |
| `framework/include/gnc/interfaces/gnc/i_guidance_3dof.hpp` | 3DOF 制导接口 | 输出攻角/滚转等飞行指令 |
| `framework/include/gnc/interfaces/gnc/i_guidance_overload.hpp` | 过载制导接口 | 为其他制导形式预留 |
| `framework/include/gnc/interfaces/gnc/i_navigation.hpp` | 导航接口 | 输出导航状态与时间戳 |
| `framework/include/gnc/interfaces/gnc/i_controller.hpp` | 控制接口 | 输出控制命令和执行器命令 |
| `framework/include/gnc/interfaces/infrastructure/i_integrator.hpp` | 积分器接口 | 仿真器与具体积分方法解耦 |
| `framework/include/gnc/interfaces/infrastructure/i_observable.hpp` | 可观测接口 | 自动记录和停止条件的基础 |
| `framework/include/gnc/interfaces/infrastructure/i_record_sink.hpp` | 记录后端接口 | 可扩展更多输出格式 |
| `framework/include/gnc/interfaces/infrastructure/i_data_logger.hpp` | 旧记录接口 | 旧版记录系统抽象 |
| `framework/include/gnc/interfaces/sensors/i_imu_sensor.hpp` | IMU 接口 | 输出 IMU 测量 |
| `framework/include/gnc/interfaces/sensors/i_gps_sensor.hpp` | GPS 接口 | 输出 GPS 测量 |
| `framework/include/gnc/interfaces/state/i_position_provider.hpp` | 位置提供接口 | 简化状态复用 |
| `framework/include/gnc/interfaces/state/i_velocity_provider.hpp` | 速度提供接口 | 简化状态复用 |
| `framework/include/gnc/interfaces/vehicle/aero_types.hpp` | 气动系数数据包 | 定义 `CL`、`CD` 等 |
| `framework/include/gnc/interfaces/vehicle/i_aero_coefficients.hpp` | 气动接口 | 动力学可据此查询气动系数 |
| `framework/include/gnc/interfaces/vehicle/i_mass_property.hpp` | 质量属性接口 | 提供质量、质心、惯量 |
| `framework/include/gnc/interfaces/propulsion/i_actuator.hpp` | 执行器接口 | 为舵面、电机等留扩展点 |
| `framework/include/gnc/interfaces/propulsion/i_engine.hpp` | 发动机接口 | 为推力与油耗模型留扩展点 |
| `framework/include/gnc/interfaces/disturbance/i_disturbance.hpp` | 扰动接口 | 为外部扰动力/力矩留扩展点 |
| `framework/include/gnc/interfaces/coord/frame_id.hpp` | 坐标系枚举 | 定义 ECI、ECEF、NED、BODY 等标识 |

## 8.6 `components/` 内建组件层

| 路径 | 作用 | 用户需要知道什么 |
| --- | --- | --- |
| `framework/include/gnc/components/README.md` | 起步件目录说明 | 明确这里是 starter components，而不是 framework core |
| `framework/include/gnc/components/_builtin_register.hpp` | 起步件汇总注册 | 默认入口包含它来触发仓库随附 starter components 的静态注册 |
| `framework/include/gnc/components/environment/wgs84_earth.hpp` | 地球模型组件 | 提供 `IEarthModel` |
| `framework/include/gnc/components/environment/spherical_gravity.hpp` | 重力模型组件 | 提供 `IGravityModel` |
| `framework/include/gnc/components/environment/standard_atmosphere.hpp` | 标准大气组件 | 提供 `IAtmosphereModel` |
| `framework/include/gnc/components/dynamics/simple_dynamics.hpp` | 简化动力学组件 | 最小可运行动力学骨架 |
| `framework/include/gnc/components/dynamics/dynamics_3dof_spherical.hpp` | 3DOF 球形地球动力学 | 更接近工程任务的动力学实现 |
| `framework/include/gnc/components/navigation/simple_navigation.hpp` | 简单导航组件 | 从动力学读取位置速度和部分姿态信息 |
| `framework/include/gnc/components/sensors/ideal_imu.hpp` | 理想 IMU 组件 | 传感器骨架，适合后续扩展误差模型 |
| `framework/include/gnc/components/state/truth_state.hpp` | 真值状态组件 | 提供位置速度真值和观测输出 |

## 8.7 `services/` 与 `libraries/` 复用层

| 路径 | 作用 | 用户需要知道什么 |
| --- | --- | --- |
| `framework/include/gnc/services/coordinate/coordinate_service.hpp` | 坐标服务 | 适合跨模块共享旋转树和坐标变换路径 |
| `framework/include/gnc/libraries/pid_controller.hpp` | PID 库 | 含抗积分饱和、微分滤波、增量式 PID |
| `framework/include/gnc/libraries/state_machine.hpp` | 状态机库 | 适合实现任务阶段逻辑、模式切换 |
| `framework/include/gnc/libraries/control/state_space.hpp` | 状态空间模型 | 连续/离散系统仿真与稳定性分析 |
| `framework/include/gnc/libraries/control/discretization.hpp` | 离散化工具 | ZOH、Tustin、Euler、后向 Euler 等 |
| `framework/include/gnc/libraries/control/tf2ss.hpp` | 控制模型转换 | 传递函数到状态空间、常见补偿环节构造 |
| `framework/include/gnc/libraries/coord/coord.hpp` | 坐标公式总入口 | 统一导出坐标旋转公式 |
| `framework/include/gnc/libraries/coord/rotations.hpp` | 坐标旋转公式 | ECEF/NED/ENU/NSE/发射系/机体系等转换 |
| `framework/include/gnc/libraries/filters/filter_base.hpp` | 滤波器接口 | 自定义滤波器和统一调用基础 |
| `framework/include/gnc/libraries/filters/butterworth.hpp` | 巴特沃斯滤波器 | 一阶低通等基础滤波能力 |
| `framework/include/gnc/libraries/filters/moving_average.hpp` | 均值类滤波器 | 滑动平均、指数平均、加权平均、中值滤波 |
| `framework/include/gnc/libraries/filters/notch.hpp` | 陷波/峰值滤波器 | 用于抑制窄带振荡和共振 |

## 8.8 非运行时辅助目录

| 路径 | 作用 | 用户需要知道什么 |
| --- | --- | --- |
| `doc/architecture.md` | 旧架构文档 | 当前已过时，不应作为现行代码依据 |
| `temp/checkTransformMatlab/` | MATLAB 验证脚本与结果 | 用于验证坐标/旋转数学实现 |
| `temp/datcom/` | 外部参考资料与源码 | 更像研究/参考材料，不属于框架运行时 API |
| `temp/*.md` | 开发过程文档 | 是框架演进记录，不是用户 API 文档 |

## 8.9 如何使用这张代码地图

如果你是：

- 使用者：优先看 `src/`、`user/`、`core/` 中的 `simulation_builder` 与 `simulator`
- 扩展者：优先看 `component_base`、`interfaces/`、`observable_helpers`、`templates/`
- 维护者：优先看 `component_factory`、`component_registry`、`config_manager`、`auto_data_logger`

只要按这个方向阅读，代码规模并不会显得杂乱。
## 8.10 Soviet Coordinate System Entry

Use these paths as the authoritative map for the coordinate-service redesign:

| Path | Role | Notes |
| --- | --- | --- |
| `framework/include/gnc/services/coordinate/coordinate_service.hpp` | Generic rotation-tree service | Pure tree, registration, LCA path composition, caching, cycle protection |
| `framework/include/gnc/services/coordinate/soviet_coordinate_system.hpp` | Built-in Soviet system formal entry | Public frame definitions, conventions, Soviet transforms, relation table, unified installer |
| `framework/include/gnc/interfaces/coord/frame_id.hpp` | Public frame ids | Official built-in frame set only: `ECI/ECEF/NUE/LAUNCH/LAUNCH_INERTIAL/BODY/TRACK/WIND` |
| `framework/include/gnc/libraries/coord/coord.hpp` | Low-level coord include | No longer the official Soviet-system entry |
| `framework/include/gnc/libraries/coord/rotations.hpp` | Low-level math helpers | Only generic axis-rotation helpers remain here |
| `tests/test_coordinate_service.cpp` | Coordinate tree + builder install regression | Verifies pure tree behavior and builder-installed Soviet system |
| `tests/test_transform.cpp` | Soviet public transform regression | Verifies Soviet formulas only through `soviet_coordinate_system.hpp` |
| `user/example_04_coordinate_service_soviet/config/mission.json` | Fixed upper-chain example | Uses only starter components |
| `user/example_05_coordinate_service_local_geographic/config/mission.json` | Dynamic NUE example | Uses a small adapter provider for geodetic `[lat, lon, alt]` |

Design boundary:

- If you accept the framework's built-in Soviet definitions, use `soviet_coordinate_system.hpp` directly.
- If your frame meanings, Euler definitions, or sign conventions differ, create a new coordinate-system implementation instead of modifying the built-in Soviet one.
