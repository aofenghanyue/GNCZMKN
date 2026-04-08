# 04 Mission 配置文件写法

## 4.1 配置文件在框架中的地位

在这套框架里，mission JSON 不是附属文件，而是“仿真装配说明书”。它决定：

- 仿真步长和总时长
- 使用哪种积分器
- 实例化哪些组件
- 每个组件叫什么
- 每个组件读取哪些参数
- 记录哪些输出
- 满足什么条件时提前终止

因此，写好配置文件和写好组件代码同样重要。

## 4.2 当前解析器支持什么，不支持什么

`ConfigManager` 使用的是轻量级自研 JSON 解析器。用户需要知道两个实际规则：

- 支持标准 JSON 中常见的对象、数组、数字、布尔值、字符串
- 不支持真正的注释语法，所以说明信息要写成 `_comment` 字段

框架在检查未使用配置项时，会自动忽略以下划线开头的键。因此模板和示例中大量使用 `_comment` 是有设计意图的，不是临时写法。

## 4.3 单飞行器任务的基本结构

最常见的结构包含三大段：

- `simulation`
- `outputs`
- `components`

最小可运行任务甚至可以只写：

- `simulation`
- `components`

## 4.4 `simulation` 段

当前实现支持的关键字段如下：

| 字段 | 含义 | 说明 |
| --- | --- | --- |
| `dt` | 仿真步长 | 主仿真步长，单位秒 |
| `duration` | 仿真时长 | 总仿真时间，单位秒 |
| `integrator` | 积分器名 | 当前支持 `rk4` 和 `euler` |
| `stop_conditions` | 停止条件数组 | 可选，满足条件可提前结束 |

### 积分器选择建议

- 大多数情况下优先用 `rk4`
- 只有在你明确需要更快但更粗糙的积分时再用 `euler`

框架对未知积分器名会给出警告，并退回 `rk4`。

### 停止条件当前支持什么

当前只支持两种：

- `component_field_below`
- `component_field_above`

每条停止条件都需要说明：

- `component`：组件实例名
- `field`：该组件的可观测字段名
- `value`：阈值
- `description`：可读描述，建议总是填写

停止条件优先使用 `IObservable` 字段；如果目标组件实现了 `IDynamicsModel`，也可以直接引用状态布局里的状态名，例如 `pos_z`、`altitude`。如果两者都不提供该字段，配置不会生效。

## 4.5 `outputs` 段

这是当前框架非常实用的一段配置。它控制自动记录行为。

支持字段如下：

| 字段 | 含义 |
| --- | --- |
| `enabled` | 是否启用自动记录 |
| `directory` | 输出目录，可包含 `{timestamp}` 占位符 |
| `format` | 当前只支持 `csv` |
| `session_name` | 输出文件名主干 |
| `precision` | 数值精度 |
| `flush_every_step` | 是否每步立即刷新文件 |
| `record` | 记录规则 |
| `exclude` | 过滤规则 |
| `debug_snapshots` | 调试快照输出规则 |

### `record` 有四种用法

#### 1. 不写 `record`

如果写了 `outputs` 但不写 `record`，框架会默认记录所有实现了 `IObservable` 的组件的所有字段。

#### 2. `record: "all"`

效果和上一种相同，语义更显式。

#### 3. `record` 写成数组

表示“只记录这些组件的全部字段”。

适合场景：

- 你已经知道只关心 `dynamics`、`guidance`、`nav`

#### 4. `record` 写成对象

这是最推荐的方式，可以为每个组件指定：

- `"all"`：记录该组件全部字段
- 字段前缀数组：只记录部分字段

例如，当前示例中就大量采用“按组件 + 按字段前缀”记录，以避免 CSV 过宽。

### `exclude` 怎么用

当前支持两类过滤方式：

- 写完整列名，排除某一个字段
- 写类似 `*.timestamp` 的模式，排除所有同名后缀字段

这在你想保留主体量、去掉冗余时间戳时很有用。

### `debug_snapshots` 怎么用

这是给 `update()` 中间量准备的一条独立调试通道，不会污染主 CSV 的稳定列结构。

推荐场景：

- 记录某一步算法中的临时误差
- 记录中间计算得到的系数、增益、切换量
- 临时排查模型内部行为，但又不想把这些量都做成长期 `IObservable` 字段

当前支持的键有：

| 字段 | 含义 |
| --- | --- |
| `enabled` | 是否启用调试快照输出 |
| `components` | 要记录哪些组件，省略或写 `"all"` 表示全部 |
| `session_name` | 调试快照文件名主干，默认是主会话名后加 `_debug_snapshots` |
| `precision` | 调试输出数值精度 |
| `flush_every_step` | 是否每步立即刷新 |

输出文件是单独的长表 CSV，固定列为：

- `time`
- `component`
- `field`
- `value`

这意味着调试字段名可以按需动态出现，不需要在仿真开始前就把所有中间量都展开成宽表列。

## 4.6 `components` 段

每个组件实例至少包含三项：

- `type`
- `name`
- `config`

### `type` 是什么

`type` 对应的是组件注册名。按当前代码习惯，它通常等于类名，例如：

- `Wgs84Earth`
- `SimpleDynamics`
- `IdealImu`
- `ConstantGuidance`

它必须出现在 `--list-components` 的结果中，否则构建会失败。

### `name` 是什么

`name` 是实例名，不是类型名。它用于：

- 依赖注入时按名字查找
- 输出字段名前缀
- 停止条件中的组件定位

所以 `name` 的稳定性很重要。推荐采用语义清晰的短名，例如：

- `earth`
- `gravity`
- `atmosphere`
- `guidance`
- `dynamics`
- `nav`

### `config` 写什么

只写该组件真正需要的参数。框架会跟踪配置项访问情况，如果你写了组件根本没读取的键，会在构建阶段给出未使用字段警告。

这是一项很有价值的特性，因为它能及时帮你发现：

- 键名拼错
- 旧配置残留
- 组件实现已经改变，但任务文件没同步更新

## 4.7 多实体配置结构

虽然当前仓库示例主要是单飞行器，但 `SimulationBuilder` 已支持：

- `global_services`
- `environment`
- `vehicles`

多实体模式的理解方法如下：

- `global_services`：全局服务
- `environment`：全局环境实体，可挂环境组件
- `vehicles`：飞行器数组，每个飞行器都有自己的 `id`、服务和组件

注册名会自动带前缀：

- 环境组件：`env.<name>`
- 飞行器组件：`<vehicle_id>.<name>`

依赖注入时：

- 同作用域内通常用短名
- 跨实体访问使用全名

## 4.8 配置命名与维护建议

### 建议做法

- 一个 mission 只表达一个任务目的
- 文件名能反映任务内容，例如 `cavh_3dof.json`、`terminal_guidance_test.json`
- 用 `_comment` 解释关键参数，不要依赖外部口头约定
- 尽量沿用现有实例命名习惯

### 不建议做法

- 在一个 mission 里混入大量历史废字段
- 用无语义名字，如 `comp1`、`moduleA`
- 让 `type` 和 `name` 同时承担不同语义，导致后期难维护

## 4.9 用仓库示例理解配置设计

当前仓库里有三种很有代表性的配置风格：

- `user/config/missions/default.json`
  作用：默认首跑任务，直接展示一条能出 CSV 的最小闭环链路
- `user/config/missions/minimal.json`
  作用：最小可运行链路
- `examples/03_cavh_3dof/cavh_mission.json`
  作用：较完整的工程化模型装配示例

阅读顺序建议是：先默认首跑，再最小配置，再完整模型。
> 2026-04-08 补充说明
>
> - `record` + `IObservable` 用于正式、稳定、长期保留的主输出字段。
> - `debug_snapshots` + `snapDebug(...)` 用于 `update()` 里的临时调试量，进入单独的长表 CSV。
> - 同一个量可以同时出现在两边，但这意味着它会被写进两份文件，而不是由框架自动去重。
> - 例如 `CavhAerodynamics` 的 `CL/CD` 既可以作为正式观测字段保留在主 CSV，也可以作为调试快照出现在 debug CSV；做任务归档时通常应按用途保留其中更合适的一边。
