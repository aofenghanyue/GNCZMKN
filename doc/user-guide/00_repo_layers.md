# 00 如何理解仓库层次

这一节不讲接口细节，只解决一个最容易让新用户误判的问题：

- 当前仓库里哪些是“框架基座”
- 哪些只是“随仓库附带的起步件”
- 哪些是“任务样板”
- 你自己的代码应该放哪

如果这一层没想清楚，后面读代码时很容易把 starter component 误当成框架协议。

## 0.1 当前最推荐的理解方式

把仓库分成四层看：

| 层次 | 当前位置 | 应如何理解 |
| --- | --- | --- |
| 基座 | `framework/include/gnc/core`、`framework/include/gnc/interfaces`、`framework/include/gnc/common` | 框架长期稳定的运行机制与接口契约 |
| 起步件 | `framework/include/gnc/components` | 随仓库附带的 starter components，用于冷启动、最小闭环和联调 |
| 样板 | `examples` | 任务级示例，展示如何用基座与起步件搭模型 |
| 用户工程区 | `user` | 用户真正应长期扩展和维护的地方 |

## 0.2 为什么这个区分现在很重要

当前仓库历史上把一批可直接运行的组件放在了 `framework/include/gnc/components` 下，这会给人一种错觉：

- 它们都属于“框架核心”
- 它们的接口与行为就是未来的官方领域标准

这两个判断都不准确。

更准确的理解是：

- 框架核心是 `core + interfaces`
- `components` 里的东西更多是“先给你一些能跑起来的件”

它们很有价值，但价值主要在：

- 冷启动
- 最小闭环
- 联调
- 教学和示例支撑

而不是在于“定义所有项目都必须遵守的领域模型”。

## 0.3 各层分别负责什么

### 基座层

负责：

- 组件生命周期
- 配置装配
- 作用域依赖注入
- 连续动力学积分
- 自动记录
- 停止条件

这里的东西，才是你应该长期依赖、长期维护的框架底盘。

### 起步件层

负责：

- 给出最小可用环境模型
- 给出最小可运行动力学骨架
- 给出最小导航与传感器占位件
- 帮用户第一时间跑通一条闭环

这里的典型组件包括：

- `SimpleDynamics`
- `SimpleNavigation`
- `IdealImu`
- `TruthState`
- `Wgs84Earth`
- `SphericalGravity`
- `StandardAtmosphere`

这些组件不是“不能长期用”，而是“不应被误解为框架的唯一正式领域标准”。

### 样板层

负责：

- 展示一类任务如何拆模
- 展示 mission 配置如何组织
- 展示自定义组件如何接入

例如：

- `examples/01_minimal`
- `examples/02_gravity_turn`
- `examples/03_cavh_3dof`

### 用户工程层

负责：

- 放你自己的组件
- 放你自己的 mission
- 放你自己的数据与输出

真正做项目时，优先扩展这里，而不是直接改 starter components。

## 0.4 当前目录为什么暂时不重组

从长期看，更理想的仓库结构会把 starter components 从 `framework/` 下面独立出去。

但当前阶段更优先的是：

- 先把边界讲清楚
- 先让文档、入口、示例和代码注释说同一种话

因为在边界还没彻底稳定前，贸然大迁目录只会放大改动面。

所以你现在应采用的心智模型是：

- `framework/include/gnc/components` 在物理路径上还在 `framework/` 下
- 但在逻辑定位上，应把它当成 starter kit 看待

## 0.5 你在不同目标下应该优先改哪里

### 如果你想理解框架怎么跑

优先看：

- `framework/include/gnc/core`
- `framework/include/gnc/interfaces`
- `src/runner.cpp`

### 如果你想快速跑通并做小改动

优先看：

- `user/config/missions/default.json`
- `user/config/missions/minimal.json`
- `user/components/`

### 如果你想学习一类完整建模方式

优先看：

- `examples/03_cavh_3dof`

### 如果你想写自己的工程组件

优先从：

- `templates/`
- `user/components/`

开始，而不是直接复制 `framework/include/gnc/components` 里的 starter components。

## 0.6 一句话结论

当前仓库最重要的阅读前提是：

- `core + interfaces` 才是基座
- `components` 是随仓库附带的起步件
- `examples` 是样板
- `user` 是你的工程入口
