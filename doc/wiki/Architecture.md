# 整体架构设计

本框架的设计初衷是提供一个**高扩展性、高内聚、低耦合**的 GNC（制导、导航、控制）仿真环境。为了实现“算法开发与基础设施分离”的目标，项目采用了经典的**五层架构**，并通过组件注册表（Registry）、工厂模式（Factory）与依赖注入（DI）串联起所有模块。

---

## 1. 五层架构模型

框架从底至上分为五大层级，各司其职，下层向上层提供支撑，上层依赖下层接口。

### 1.1 数学库层 (Math Library)
*   **位置**：`framework/include/gnc/common/math/`
*   **职责**：这是最底层的纯数学运算库，所有上层物理计算的基础。它对 `Eigen3` 进行了领域定制化封装，提供 GNC 领域高频使用的数学工具。
*   **核心内容**：
    *   **`quaternion.hpp` / `rotation.hpp`**：四元数及旋转矩阵操作。
    *   **`calculus.hpp`**：数值微积分方法，包括 RK4（四阶龙格-库塔）积分器等。
    *   **`interp.hpp`**：提供一维/二维插值算法，对于处理气动数据表极为重要。

### 1.2 算法库层 (Algorithm Library)
*   **位置**：`framework/include/gnc/libraries/`
*   **职责**：面向 GNC 领域的通用算法模块集合，与具体飞行器无关。
*   **核心内容**：
    *   **`pid_controller.hpp`**：标准的增量/位置式 PID 算法实现。
    *   **`filters/`**：包含巴特沃斯、陷波、滑动平均等数字滤波器。
    *   **`coord/rotations.hpp`**：提供纯数学的坐标系静态转换公式（如 ECEF 转 NED、机体转风系等）。

### 1.3 接口层 (Interface Layer)
*   **位置**：`framework/include/gnc/interfaces/`
*   **职责**：框架的**骨架**，定义了组件之间互相通信的标准契约。它强制规定了某类组件“必须提供什么能力”。
*   **核心内容**：
    *   **环境接口**：`IAtmosphereModel`（大气）, `IGravityModel`（重力）, `IEarthModel`（地球）。
    *   **GNC接口**：`IGuidance3DOF`（三自由度制导）, `INavigation`, `IController`。
    *   **动力学与状态**：`IDynamicsModel`（动力学积分模型）, `IAeroCoefficients`（气动系数提供者）, `IMassProperty`（质量特性）。
    *   **基础设施接口**：`IObservable`（可观测变量接口，用于日志记录）。

### 1.4 服务层 (Service Layer)
*   **位置**：`framework/include/gnc/services/`
*   **职责**：提供**跨组件的全局设施**，这类设施通常不是以单独的“飞行器组件”存在，而是作为基础设施供所有组件调用。
*   **核心内容**：
    *   **`CoordinateService`**：动态树形坐标变换管理。组件无需手写繁琐的变换链，只需向服务请求 `BODY` 到 `ECEF` 的变换，服务通过图的 LCA 算法自动合成转换矩阵，并带有节点级缓存。

### 1.5 应用层 (Application Layer)
*   **位置**：`framework/include/gnc/core/` 及 `framework/include/gnc/components/`
*   **职责**：框架的**运行引擎**和**具体组件实现**。
*   **核心内容（引擎）**：
    *   **`ComponentBase`**：所有自定义组件的基类。
    *   **`ComponentFactory` & `ComponentRegistry`**：负责根据 JSON 中的字符串名称，反射实例化对象并加入注册表。
    *   **`Simulator`**：仿真大循环控制器，负责时间步的推进和按频率调度组件的 `update()` 方法。
*   **核心内容（内置组件）**：如 `SimpleDynamics`, `StandardAtmosphere` 等现成组件。

---

## 2. 核心架构设计模式

本框架能够做到仅依靠一个 JSON 文件就驱动任意组合的组件，得益于以下几种核心设计模式：

### 2.1 工厂模式 + 宏自动注册
C++ 本身不支持基于字符串的反射。为了让 JSON 配置中的 `"type": "CavhMass"` 能够实例化为 C++ 对象，框架在 `component_factory.hpp` 中实现了一个单例工厂。
通过宏 `GNC_REGISTER_COMPONENT(ClassName, InterfaceType)`，组件在编译期会自动将自身的构造函数注册到工厂的 Hash Map 中。

### 2.2 依赖注入 (Dependency Injection)
组件不需要在内部 `new` 出自己依赖的对象。相反，组件只需在 `injectDependencies(ScopedRegistry& registry)` 阶段，向注册表“索要”自己需要的接口指针。
例如，气动组件只需要说“我需要一个 `IDynamicsModel` 接口来获取马赫数”，注册表就会把动力学组件的指针注入给它。这实现了极高的解耦。

### 2.3 基于接口编程
上层 `Simulator` 和各个组件之间互不认识具体类型。动力学积分器并不关心为它提供气动力的是 `CavhAerodynamics` 还是基于查表的 `TabulatedAero`，它只认识 `IAeroCoefficients` 接口。

### 2.4 作用域注册表 (Scoped View)
为了支持**多飞行器对抗仿真**（如导弹打飞机），框架设计了全局的 `ComponentRegistry` 和针对单个飞行器的视图 `ScopedRegistry`。
不同飞行器的同名组件（如 `missile.dynamics` 和 `target.dynamics`）在全局注册表中被加上了命名空间前缀，但在组件内部代码中，依然只需通过 `registry.getByName("dynamics")` 即可安全地拿到属于自己飞行器的组件。

---
> 下一步：了解这套架构运转起来后，数据是如何在各个组件间流动的，请阅读 [**数据流向与工作流程**](DataFlow_Workflow.md)。
