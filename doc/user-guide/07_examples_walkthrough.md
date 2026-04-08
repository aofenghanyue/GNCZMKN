# 07 示例与现有任务讲解

## 7.1 为什么一定要读示例

这套框架的设计思想虽然清晰，但只看接口层容易停留在抽象上。示例的价值在于把抽象落实成真实任务装配路径，告诉你：

- 任务配置该怎么写
- 组件应该拆到什么粒度
- 最小模型和工程模型之间差在哪

## 7.2 示例一：最小可运行链路

相关文件：

- `examples/01_minimal/README.md`
- `examples/01_minimal/components/constant_guidance.hpp`
- `examples/01_minimal/config/mission.json`

这个示例演示的是最小工作闭环：

- 常值制导产生固定加速度
- 简化动力学推进状态
- IMU 和导航从动力学读取状态

你应该从它学到的不是“这个制导算法怎么写”，而是：

- 一个用户组件如何实现
- 一个 mission 如何实例化组件
- 一条最简单的 GNC 链路如何在框架中表达

## 7.3 仓库当前用户入口：`minimal.json`

相关文件：

- `user/components/guidance/constant_guidance.hpp`
- `user/config/missions/minimal.json`

这实际上是把“最小示例”搬到了用户工作区里，方便直接构建和运行。它的意义非常大，因为它天然就是你做第一次自定义修改的起点。

推荐做法：

1. 先直接运行它
2. 只改制导参数
3. 再改字段记录
4. 最后再替换成自己的制导类

## 7.4 示例二：Gravity Turn

相关文件：

- `examples/02_gravity_turn/simple_gravity_turn.cpp`

这个示例的特点是：

- 不依赖外部 mission 文件
- 直接在程序内部构造 JSON 字符串
- 组件本身实现了 `IObservable`
- 运行后通过自动记录得到 `gravity_turn.csv`

这个示例说明了两点：

- `SimulationBuilder` 不一定只能读文件，也可以读字符串配置
- 框架既可以作为“统一入口程序”，也可以作为“被其他专用程序嵌入的仿真内核”

适合用它做的事情：

- 快速算法实验
- 小型专用验证程序
- 不想先维护 mission 文件时的原型验证

## 7.5 示例三：CAV-H 3DOF

相关文件：

- `examples/03_cavh_3dof/main.cpp`
- `examples/03_cavh_3dof/cavh_mission.json`
- `examples/03_cavh_3dof/cavh_programmed_aoa.hpp`
- `examples/03_cavh_3dof/cavh_aerodynamics.hpp`
- `examples/03_cavh_3dof/cavh_mass.hpp`

这是当前仓库里最有工程价值的示例。它展示了如何把一个较完整的飞行器模型拆成多个角色清晰的组件：

- `CavhProgrammedAoA`
  作用：按高度程序给出攻角和滚转指令
- `CavhAerodynamics`
  作用：根据攻角和马赫数提供气动系数
- `CavhMass`
  作用：提供质量属性
- `Dynamics3DOF_SphericalEarth`
  作用：综合环境、质量、气动和制导推进状态

你应该重点学习它的两点：

- 如何把领域模型拆成多个独立组件
- 如何通过 `outputs.record` 只记录真正关键的字段

当前这个示例还额外演示了 `outputs.debug_snapshots`：

- `cavh_mission.json` 为 `aero` 组件开启了调试快照
- `CavhAerodynamics` 在 `update()` 中用 `snapDebug()` 记录 `alpha_rad`、`speed`、`mach`、`CL`、`CD`
- 输出目录里会多出一份独立调试 CSV，适合排查气动中间量，而不必把这些量全部塞进主观测字段

## 7.6 CAV-H 示例透露出的建模方法

这个示例非常适合作为“如何从任务需求拆模型”的范本。它没有把所有公式塞进一个巨大类，而是按物理职责拆分：

- 环境独立
- 质量独立
- 气动独立
- 制导独立
- 动力学汇总

这正是这套框架最值得坚持的使用方式。

## 7.7 示例之间的递进关系

这三个示例实际上构成了一个非常好的学习路径：

### 第一层：验证框架主链

用最小示例理解：

- 自动注册
- mission 装配
- 运行入口

### 第二层：理解框架可嵌入性

用 gravity turn 理解：

- `loadConfigString()`
- 组件自定义观测字段
- 专用程序嵌入式使用

### 第三层：理解工程化拆模

用 CAV-H 理解：

- 多组件协同
- 领域角色拆分
- 可维护的任务配置

## 7.8 如何把示例改造成自己的工程起点

### 如果你的任务只是快速闭环验证

从最小示例出发：

- 保留 `SimpleDynamics`
- 改制导
- 改输出

### 如果你的任务是大气飞行或再入

从 CAV-H 示例出发：

- 保留 3DOF 动力学骨架
- 替换气动模型
- 替换质量模型
- 替换制导律

### 如果你的任务需要独立程序控制实验流程

从 gravity turn 出发：

- 自己写专用 `main`
- 直接嵌入 JSON 字符串或生成配置
- 继续复用 `SimulationBuilder`

## 7.9 示例之外，测试也值得参考

测试目录不是教程，但它能补足几个关键认知：

- `test_cross_entity_access.cpp` 说明跨实体全名访问是支持的
- `test_integrators.cpp` 验证 RK4/Euler 的基本行为
- `test_transform.cpp` 说明数学/坐标库经过了数值验证
- `test_math_headers.cpp` 说明数学扩展头文件是可单独编译复用的

因此，示例告诉你“怎么用”，测试告诉你“作者认为哪些行为必须成立”。
> 2026-04-08 补充说明
>
> - CAV-H 示例里，`CavhAerodynamics` 既可以把 `CL/CD/lift_to_drag` 作为正式观测字段暴露给主 CSV，也可以在 `update()` 里用 `snapDebug()` 记录调试快照。
> - 如果两边同时开启，主 CSV 和 debug CSV 会各保留一份相同物理量；这是两条通道都在工作，不是框架错误。
> - 用法建议是：长期分析和任务归档优先看正式观测字段，单步排错和调参优先看 `debug_snapshots`。
