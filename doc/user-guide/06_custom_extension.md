# 06 如何实现自定义扩展

## 6.1 先理解扩展目标

在这个框架里，自定义扩展的推荐路径不是修改核心框架代码，而是：

1. 先确定组件在仿真链路中的职责
2. 选择合适的接口或接口组合
3. 在 `user/components/` 下实现自己的组件
4. 用 mission 配置把它装配进任务

这样做的核心收益是，框架核心保持稳定，而你的领域模型、算法模型和任务模型都留在用户工作区里演化。

## 6.2 先选接口，再写类

不要先想“我要写一个什么名字的类”，而要先想“这个组件对外扮演什么角色”。

常见角色包括：

- 制导：`IGuidance`、`IGuidance3DOF`、`IGuidance6DOF`
- 导航：`INavigation`
- 控制：`IController`
- 动力学：`IDynamicsModel`
- 状态暴露：`IPositionProvider`、`IVelocityProvider`、`IAltitudeProvider`
- 传感器：`IImuSensor`、`IGpsSensor`
- 环境：`IAtmosphereModel`、`IGravityModel`、`IEarthModel`
- 飞行器属性：`IAeroCoefficients`、`IMassProperty`
- 正式输出：`IObservable`

建议把“角色接口”和“内部实现”分开看。组件对外暴露的是角色，不是内部状态布局、缓存结构或字符串字段名。

## 6.3 一个标准组件应包含什么

一个可维护的组件通常包含下面这些部分：

- 构造函数：设置组件名、默认执行频率、初始化内部状态
- `configure()`：读取任务配置中的参数
- `injectDependencies()`：获取其他组件接口
- `injectServices()`：获取服务型依赖
- `initialize()`：做一致性检查、准备缓存
- `update()` 或 `computeDerivatives()`：实现核心算法
- `getObservableFields()`：暴露正式记录字段

连续动力学组件是一个特殊情况：

- 如果实现了 `IDynamicsModel`，连续状态由积分器统一推进
- `computeDerivatives()` 负责计算状态导数
- `update(dt)` 更适合做积分后的缓存刷新、派生量计算和调试快照写入
- 不建议在 `update(dt)` 里再次自行推进连续状态

## 6.4 组件应该放在哪里

推荐把自定义组件放在 `user/components/` 下，并按职责分目录，例如：

```text
user/components/
├── guidance/
├── navigation/
├── controller/
└── vehicle/
```

当前构建系统会自动扫描 `user/components/**/*.hpp`，因此通常不需要修改顶层 `CMakeLists.txt`。

## 6.5 如何注册组件

每个组件头文件末尾都应使用注册宏，把“类名”和“它实现的角色接口”注册到工厂中。

建议遵循两条规则：

- mission 中的 `type` 直接等于类名
- 一个头文件只放一个组件

这样最容易维护，也最不容易在装配阶段出错。

## 6.6 依赖注入的推荐写法

### 同作用域依赖

大多数情况下，组件只依赖同一个飞行器或同一个环境上下文内的其他组件。此时推荐在 `injectDependencies()` 里使用短名，例如：

- `guidance`
- `dynamics`
- `imu`
- `nav`

框架会根据当前组件所属作用域自动补齐前缀。

### 跨作用域依赖

如果你明确需要访问其他实体的组件，应使用全名：

- `target.dynamics`
- `env.gravity`

这是一条显式通道，而不是默认行为。框架不会把其他作用域的组件偷偷混进 `getAll<Interface>()` 的结果里。

### 批量绑定辅助接口

当前推荐使用：

- `gnc::core::bind(member_ptr, "name")`
- `registry.bindAll(...)`

这组接口的目的不是改变依赖语义，而是减少样板代码。它特别适合一个组件需要同时绑定多个依赖时的场景。
现在 `bind(...)` 还额外表达了“这是一个必需依赖”的语义。即使组件暂时没有实现 `IDependencyDeclarer`，构建器也会在依赖预检查阶段尝试执行这类绑定，并把缺失依赖提前暴露为构建错误，而不是把问题留到仿真初始化甚至运行期。

## 6.7 如何声明“必需依赖”和“可选依赖”

如果某个依赖缺失会让组件失去意义，建议实现 `IDependencyDeclarer`，并在 `getDependencies()` 中显式声明。

但这不再是“复制模板后的第一步必做项”。当前模板默认先用 `bind(...)` / `bindIfPresent(...)` 起步，由构建期预检查先把缺失的必需依赖拦下来。

当前仓库自己的组件也已经开始按这条边界收口：

- `SimpleNavigation`、`TruthState`、`CavhProgrammedAoA` 这类轻量组件，已经不再为少量同作用域必需绑定单独维护 `getDependencies()`
- `Dynamics3DOF_SphericalEarth`、`CavhAerodynamics` 这类多依赖装配型组件，仍然保留显式依赖声明

还要注意一条现在已经生效的规则：

- `IDependencyDeclarer` 不会替代真实的 `injectDependencies()`
- 显式声明通过后，框架仍然会继续对真实绑定逻辑做预检查

这样可以避免一种很隐蔽的问题：

- `getDependencies()` 写对了
- 但 `injectDependencies()` 因为拼写漂移或后续改动绑定错了名字
- 结果以前只能到初始化甚至运行期才暴露

现在这类“声明和真实绑定脱节”的问题也会继续在构建期被拦下。

当前推荐使用：

- `requireDependency<Interface>("name", "description")`
- `optionalDependency<Interface>("name", "description")`

这里的 `name` 不是装饰信息，而是框架会真正拿来做解析和校验的查找名。

这意味着依赖校验现在不再只是检查“全局某处有没有这个接口”，而是会检查：

- 该组件按自己的作用域去解析时，会落到哪个名字
- 这个名字是否存在
- 这个名字对应的组件是否真的实现了目标接口
- 同作用域里有哪些候选组件
- 全局有哪些实现了该接口的组件

这对多实体任务尤其重要。比如 `missile.nav` 依赖 `dynamics` 时，框架现在会检查 `missile.dynamics`，而不会因为 `target.dynamics` 恰好存在就误判为通过。

## 6.8 什么时候应该继续抽 typed provider

不要把所有量一开始都做成接口，但也不要把已经稳定的业务依赖长期留在字符串字段访问层。

一个实用判断标准是：

- 如果某个量只是为了记录、诊断、临时停机判据而暴露，字符串字段访问就足够
- 如果某个量开始成为另一个组件算法的正式输入，就应考虑把它抽成 provider 接口

当前仓库里的一个真实例子是：

- `CavhProgrammedAoA` 需要高度来查表生成攻角指令
- 这个依赖现在通过 `IAltitudeProvider` 表达
- 而不是继续使用 `getStateValue("altitude")`

这类接口的价值不在于“接口越多越好”，而在于把已经稳定下来的业务耦合关系从隐式字符串约定提升为显式类型契约。

## 6.9 什么时候应该声明依赖

推荐规则如下：

- 缺了就完全不能工作的依赖：至少先写成 `bind(...)`，在依赖关系稳定后补 `requireDependency`
- 缺了仍能降级运行的依赖：声明为 `optionalDependency`
- 只是临时调试或实验用途的弱依赖：可以先不声明，但最好尽快收敛

可以再加一个更实用的判断：

- 如果 `getDependencies()` 只是逐行重复 `injectDependencies()` 里的 1 到 2 个同作用域必需绑定，通常可以先不写
- 如果组件是一个真正的装配汇聚点，缺失依赖时希望一次性列出整组问题，就值得继续写 `IDependencyDeclarer`

不要把 `IDependencyDeclarer` 理解成“必须实现的繁琐步骤”。它的价值在于把运行期空指针问题提前为构建期诊断问题。
即便如此，对关键依赖仍然推荐显式声明，因为 `IDependencyDeclarer` 给出的错误信息会更完整，也更适合多实体装配诊断。

## 6.10 服务依赖和组件依赖的区别

组件依赖适合表达“某个实体内部或某个明确实体上的仿真角色关系”，例如：

- 导航依赖 IMU
- 动力学依赖气动和质量模型
- 控制依赖制导命令

服务依赖适合表达“跨多个组件共享的基础设施能力”，例如：

- 坐标变换
- 星历
- 全局时钟
- 统一场景服务

不要把服务当作业务状态容器来滥用。飞行器自身状态、制导命令、导航解算结果等，更适合继续放在组件接口体系里。

## 6.11 正式输出和调试输出要分层

框架现在明确区分两类输出：

### 正式输出

通过 `IObservable` 暴露，适合：

- 主 CSV 记录
- stop condition
- 任务后分析
- 长期稳定使用的字段

推荐用 `ObservableFieldBuilder` 组织字段。

### 调试输出

通过 `ComponentBase::snapDebug(name, value)` 写入，适合：

- `update()` 过程中的中间量
- 临时检查量
- 不希望污染正式输出模型的字段

如果在 mission 中启用了 `outputs.debug_snapshots`，这些调试量会写入独立 CSV，而不是混进主记录表。

## 6.12 字段设计建议

无论是 `IObservable` 字段还是调试快照字段，都建议遵循这些原则：

- 名称直接表达物理意义
- 尽量能脱离源码上下文独立理解
- 对外字段名不要跟随内部变量名摇摆
- 单位要在文档或配置注释中说清楚

好的字段名示例：

- `altitude`
- `mach`
- `alpha_deg`
- `lift_to_drag`
- `position.x`

## 6.13 stop condition 应依赖什么字段

stop condition 优先复用 `IObservable` 字段体系；如果目标组件实现了 `IDynamicsModel`，也可以直接按状态名读取连续状态字段。因此，任何可能成为终止判据的量，都应该优先成为正式输出字段或明确的动力学状态名，而不是只写到 `snapDebug()`。

典型终止字段包括：

- 高度
- 速度
- 质量
- 误差量
- 能量量

## 6.14 推荐开发流程

推荐按下面顺序开发新组件：

1. 从 `templates/` 复制最接近的模板
2. 改类名、注册名和接口
3. 先定义配置参数
4. 再定义依赖注入
5. 先用 `bind(...)` / `bindIfPresent(...)` 跑通最小链路
6. 再为关键依赖补 `IDependencyDeclarer`
7. 先在最小任务里单独验证组件行为
8. 再补正式输出字段
9. 最后再放进完整任务链路

这套顺序的核心目的是把“算法问题”和“装配问题”拆开，降低排错成本。

## 6.15 当前模板文件各自适合什么

| 模板 | 适用场景 |
| --- | --- |
| `templates/guidance_template.hpp` | 新制导组件起点 |
| `templates/navigation_template.hpp` | 新导航组件起点 |
| `templates/controller_template.hpp` | 新控制组件起点 |
| `templates/mission_template.json` | 新任务配置起点 |

这些模板现在已经采用：

- `bindAll(...)` 批量依赖绑定
- 默认不强制 `IDependencyDeclarer`
- 在需要更强装配诊断时再补 `requireDependency(...)` / `optionalDependency(...)`

因此它们不只是“类结构示例”，而是当前框架推荐扩展姿势的最小样板：先把组件角色和依赖绑定写清楚，再决定是否需要更重的显式依赖声明。

## 6.16 常见错误

### 只写了类，没有注册

结果是 mission 中永远找不到这个 `type`。

### 依赖写短名，但任务实际在别的作用域

结果是构建时会被新的依赖校验直接拦下，而不是运行后才出空指针问题。

### 把正式输出和调试输出混在一起

结果是主 CSV 变脏，stop condition 语义也会变混乱。

### 在动力学组件的 `update()` 中自己重复推进连续状态

结果会和积分器路径冲突，或让组件调度语义变得不清晰。

### 用字符串字段访问替代明确的业务接口

字符串字段访问适合通用记录和 stop condition，不适合作为高频业务耦合通道。组件间算法依赖仍应优先使用 typed interface。

## 6.17 这一章最重要的结论

如果只记一件事，请记住下面这条：

**先用接口把角色讲清楚，再用 mission 把组件装起来；缺少依赖时，让构建期诊断替你兜底，而不是把问题留到运行期。**
> 2026-04-08 补充说明
>
> - 正常 mission 启动时，`SimulationBuilder::build()` 会先执行一次真实 `injectDependencies()` 预检查；`Simulator::initialize()` 只会补做那些尚未完成的注入。
> - 因此 `injectDependencies()` 应尽量只表达绑定关系，避免把一次性副作用初始化混进去；更适合放在 `initialize()` 的内容包括缓存准备、一致性检查和昂贵预计算。
> - `IObservable` / `record` 与 `snapDebug(...)` / `debug_snapshots` 是两条不同输出通道：前者面向稳定主输出，后者面向临时调试。
> - 如果同一个量两边都开，用户会在主 CSV 和调试 CSV 里各看到一份记录；例如 `CavhAerodynamics` 里的 `CL/CD` 就可能出现这种“双通道并存”。
> - 当前仓库已经不再依赖重复 declarer 才能获得构建期整组缺依赖诊断；`IDependencyDeclarer` 的主要价值已收敛到“显式依赖契约和语义说明”。
