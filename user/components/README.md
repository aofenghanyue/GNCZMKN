# 用户组件目录

把你自己的组件放在这里，或放在这里的子目录里。这里是用户工程区，不是框架基座。

## 推荐结构

```text
user/components/
|-- guidance/
|-- navigation/
|-- controller/
`-- vehicle/
```

## 当前推荐工作流

1. 从 `templates/` 复制最接近的模板。
2. 修改类名、文件名和注册名。
3. 在 `configure()` 中读取任务参数。
4. 在 `injectDependencies()` 中用 `registry.bindAll(...)` 绑定依赖。
5. 对“存在则增强、不存在也能合法降级”的依赖，优先使用 `bindIfPresent(...)`。
6. 先把最小链路跑通，再为关键依赖补充 `IDependencyDeclarer + requireDependency(...)`。
7. 为需要正式记录的输出，实现 `IObservable`。
8. 只用于中间调试的量，使用 `snapDebug(...)`。

## 什么时候将状态提取为 typed provider

如果某个量只是为了日志、诊断或 stop condition，字段访问通常就够了。

如果某个量开始反复成为别的组件算法输入，就应该考虑把它提升成 typed provider。当前仓库里的真实例子包括：

- `IAltitudeProvider`：让 `CavhProgrammedAoA` 不再依赖 `getStateValue("altitude")`
- `IVelocityProvider`：让 `CavhAerodynamics` 不再依赖字符串状态名
- `IAttitudeProvider` / `IAngularVelocityProvider`：让导航、真值和 IMU 不再偷看动力学状态布局

## 依赖注入建议

- 同作用域依赖优先用短名，例如 `guidance`、`dynamics`、`nav`
- 跨实体访问用全名，例如 `target.dynamics`
- `bind(...)` 用于必需依赖
- `bindIfPresent(...)` 用于合法可选依赖
- 如果组件没有实现 `IDependencyDeclarer`，`bind(...)` 缺失时构建器也会在依赖预检查阶段直接报错
- 但对关键依赖，仍然推荐在依赖关系稳定后显式写 `IDependencyDeclarer + requireDependency(...)`
- 如果某个组件只有 1 到 2 个同作用域必需绑定，而且 `getDependencies()` 只是在重复这些绑定，通常没必要急着补 declarer
- 即使实现了 `IDependencyDeclarer`，框架也仍会继续预检查真实 `injectDependencies()` 绑定是否和声明一致
- `injectDependencies()` 应尽量保持为无副作用的绑定阶段；一次性初始化更适合放在 `initialize()`
- `ScopedRegistry::getAll<Interface>()` 只返回当前作用域组件

## 自动发现规则

- 每个组件建议放在一个独立 `.hpp` 文件里
- 顶层构建会自动扫描 `user/components/**/*.hpp`
- 文件放好后重新执行 `cmake` 和构建即可

## 注册规则

每个组件文件末尾都需要注册宏，例如：

- `GNC_REGISTER_COMPONENT(MyGuidance, gnc::interfaces::IGuidance)`

mission 里的 `type` 应直接对应注册名。最稳妥的做法是让注册名与类名一致。

## 与 starter components 的关系

`framework/include/gnc/components/` 里的组件是仓库随附的 starter components，适合作为：

- 默认任务
- 最小闭环
- 示例链路
- 起步模板

如果你在做自己的工程扩展，应优先在 `user/components/` 中演化自己的模型，而不是长期直接修改 starter components。

## 模板入口

见 `templates/` 目录。
> 2026-04-08 补充说明
>
> - 正常 mission 路径下，`SimulationBuilder::build()` 会先做一次真实依赖预检查；`Simulator::initialize()` 只会补做尚未完成的注入。
> - 因此 `injectDependencies()` 应尽量保持为无副作用的绑定阶段；一次性初始化更适合放在 `initialize()`。
> - 现在即使不实现 `IDependencyDeclarer`，`bind(...)` 缺失也会在构建期被直接拦下。
> - `IDependencyDeclarer` 更适合在你确实需要显式依赖契约时再补，而不是复制模板后的默认第一步。
