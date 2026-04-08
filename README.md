# GNC 仿真框架

这是一个以 C++17、Eigen3、头文件组件扩展和 JSON 任务装配为核心的 GNC 仿真框架。

它的重点不是预置所有业务模型，而是把下面这些基础设施固定下来：

- 组件生命周期与频率调度
- 连续动力学积分接口
- 配置驱动装配
- 作用域依赖注入
- 自动数据记录与停止条件

## 先看哪里

- 用户手册入口：`doc/user-guide/README.md`
- 运行入口：`src/runner.cpp`
- 用户工作区：`user/active_project`、`user/<project>/components/`、`user/<project>/config/`
- 仓库随附样板工程：`user/example_*`
- 模板：`templates/`

## 当前仓库的层次应该怎么理解

### `framework/include/gnc/core`

这是框架真正的运行时基座，负责：

- 组件基类
- 组件工厂与注册表
- 配置解析
- 仿真器与构建器
- 自动记录

### `framework/include/gnc/interfaces`

这是接口契约层，负责定义：

- 动力学
- 环境
- 制导/导航/控制
- 传感器
- 状态提供
- 输出与积分器

### `framework/include/gnc/components`

这里虽然目前放在 `framework/` 下面，但应当把它理解成：

- 随仓库附带的起步件
- 冷启动用的基础实现
- 用于最小闭环、联调和示例支撑的 starter components

它们不是“框架核心协议”，也不是所有项目都应长期依赖的领域标准件。
只有已经在多个工程之间复用、语义稳定、适合作为仓库公共起步件的组件，才应放在这里；项目私有或仍在快速演化的组件，优先留在 `user/<project>/components/`。

### `user/example_*`

这里放的是随仓库附带的样板工程，用来说明：

- 如何组织一套模型
- 如何拆分角色
- 如何写 mission

### `user`

这里是用户工作区，才是最推荐的扩展位置：

- `user/<project>/components/` 放该工程的自定义组件
- `user/<project>/config/` 放该工程的任务配置
- `user/active_project` 指定当前活跃工程

如果你的目标是“基于框架做自己的工程”，优先改这里，而不是直接改 `framework/include/gnc/components/`。

## 当前默认入口

直接运行 `gnc_sim` 时，默认读取：

- `user/example_03_cavh_3dof/config/mission.json`（由当前提交的 `user/active_project` 决定）

这个文件现在是仓库默认提交的首跑工程任务，可以直接跑通并输出 CSV。
运行器会从当前工作目录和可执行文件目录向上搜索这条相对路径，因此从仓库根目录或 `build/bin` 直接启动都可以。
对这类 repo-relative mission，运行器还会把相对输出路径继续锚定在同一个项目根下，因此结果仍然落到仓库里的 `user/outputs/`。

如果你要运行别的任务，优先使用：

- `gnc_sim --config user/example_01_minimal/config/mission.json`

注意：

- `--config` 只负责解析任务路径；示例/项目专用组件是否可用，取决于构建期选择的自定义组件根
- 当前仓库已把 project-style 示例并入 `user/` 工作区，例如 `user/example_01_minimal` 和 `user/example_03_cavh_3dof`
- `user/active_project` 是仓库内的活跃工程文件，修改它后重新执行 `cmake --build`，CMake 会自动先重配置再构建

## 快速自检

如果你想先确认框架当前识别到了哪些 starter/custom 组件，可以运行：

- `gnc_sim --list-components`
- `gnc_sim --list-components-verbose`

其中 verbose 形式会额外显示每个类型的注册来源头文件。

## 当前最重要的边界

如果只记一条，请记这条：

- `core` 和 `interfaces` 是长期基座
- `framework/include/gnc/components` 是仓库共享起步件
- `user/example_*` 是随仓库附带的样板工程
- `user/<your_project>` 才是你的工程入口
