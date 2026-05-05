# Mission 配置

mission 是 GNCZMKN 的装配入口。当前有效 schema 使用顶层 `vehicles[]`，单飞行器任务就是数组中只有一个条目。

## 顶层结构

```json
{
  "simulation": {
    "dt": 0.1,
    "duration": 600.0,
    "integrator": "rk4"
  },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "form": { "components": [] },
      "perturbation": {
        "type": "perturbation.static",
        "name": "perturbation",
        "config": { "inputs": {} }
      },
      "services": {},
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
    }
  ],
  "outputs": {},
  "termination": {
    "type": "termination.component_field_below",
    "name": "termination",
    "config": {}
  }
}
```

旧式 `entities[]`、根级 `components/services`、根级 `form/vehicle/interaction` 不属于当前 schema。
`termination` 和 `summary` 都是可选顶层单例；上面的 `termination.config` 只是结构占位，真实任务需要填写被监控组件、字段和阈值。

`global_services` 是预留的高级顶层服务块；当前内置 `coordinate_tree` 只支持 vehicle scope，因此普通 mission 不应使用顶层 `global_services`。

## 命名和作用域

每个 vehicle 必须有 `id`。合法 id 匹配 `[A-Za-z_][A-Za-z0-9_-]*`，并且不能是 `env`。

运行时组件名由作用域前缀和组件 `name` 组成：

| Mission 位置 | 示例 name | 完整组件名 |
| --- | --- | --- |
| `environment.components` | `earth` | `env.earth` |
| `vehicles[0].form.components`，vehicle id 为 `vehicle` | `dynamics` | `vehicle.dynamics` |
| `vehicles[0].process`，vehicle id 为 `vehicle` | `guidance` | `vehicle.guidance` |
| `termination` | `termination` | `mission.termination` |

`outputs.record`、termination 的 `component`、跨作用域依赖配置都应使用完整组件名。组件内部的默认依赖名通常是当前 vehicle 作用域内的短名，例如 `dynamics`、`interaction`、`mass`、`aero`；环境依赖通常使用 `env.*`。

## Include and Merge

`ConfigManager::loadFromFile()` 会在 mission assembly 前展开 `$include`。`$include` 可以是 string 或有序 string array：

```json
{
  "$include": "project://config/fragments/environment.json",
  "components": []
}
```

```json
{
  "$include": [
    "project://config/presets/base_vehicle.json",
    "user-data://vehicles/cavh/defaults.json"
  ],
  "id": "vehicle"
}
```

include 路径默认相对当前 JSON 文件解析；显式 root 如下：

| Prefix | Root |
| --- | --- |
| `repo://` | 仓库根目录 |
| `project://` | 当前 `user/<project>/` 根目录 |
| `user-data://` | `user/data/` |

多个 include 按顺序 deep merge；本地字段覆盖 include 字段；数组整体替换。`loadFromString()` 没有文件系统锚点，因此会拒绝 `$include`。

## Simulation

| 字段 | 规则 |
| --- | --- |
| `dt` | 必填，固定步长，必须 `> 0` |
| `duration` | 必填，必须 `>= 0` |
| `integrator` | 可省略，默认 `rk4`；当前支持 `rk4` 和 `euler` |

`duration / dt` 必须接近整数，否则 build 失败。未知 integrator 不回退，直接 build error。

## Environment

环境块由可选 `services` 和 `components` 组成。内置环境组件通常放在 `environment.components`：

```json
{
  "environment": {
    "components": [
      {
        "type": "environment.spherical_earth",
        "name": "earth",
        "config": {}
      },
      {
        "type": "environment.standard_atmosphere",
        "name": "atmosphere",
        "config": {}
      },
      {
        "type": "environment.spherical_gravity",
        "name": "gravity",
        "config": {}
      }
    ]
  }
}
```

环境组件注册到 `env.<name>`。例如 local-spherical point-mass 默认查找 `env.earth`，aero-propulsive interaction 默认查找 `env.atmosphere` 和 `env.gravity`。

## Form 示例

Cartesian 3DoF：

```json
{
  "type": "form.cartesian_3dof.point_mass",
  "name": "dynamics",
  "config": {
    "initial_position": [0.0, 0.0, 1000.0],
    "initial_velocity": [250.0, 0.0, 0.0]
  }
}
```

Local-spherical 3DoF：

```json
{
  "type": "form.local_spherical_3dof.point_mass",
  "name": "dynamics",
  "config": {
    "launch_azimuth_rad": 1.5707963267948966,
    "initial_state": {
      "longitude_rad": 1.9198621771937625,
      "latitude_rad": 0.5235987755982988,
      "altitude_m": 60000.0,
      "speed_mps": 3200.0,
      "flight_path_angle_rad": -0.1047197551196598,
      "heading_angle_rad": -1.5707963267948966
    }
  }
}
```

初始状态是物理配置，缺失或类型错误会 build fail。

## Vehicle 和 Interaction

`vehicle.process` 适合放导航、制导、控制和命令生成；`vehicle.output` 适合放气动、质量、推进等运行时物理能力。

```json
{
  "process": [
    {
      "type": "vehicle.process.programmed_aoa",
      "name": "guidance",
      "config": {
        "bank_angle_deg": 0.0,
        "schedule_altitude_m": [60000, 45000, 30000, 15000],
        "schedule_angle_of_attack_deg": [20, 12, 10, 8]
      }
    }
  ],
  "output": [
    {
      "type": "mass.constant",
      "name": "mass",
      "config": {
        "asset_file": "framework/data/vehicles/cavh/output/mass_atmospheric_reference.json"
      }
    },
    {
      "type": "aero.table2d",
      "name": "aero",
      "config": {
        "asset_file": "framework/data/vehicles/cavh/output/aero_table2d.json"
      }
    }
  ],
  "interaction": {
    "components": [
      {
        "type": "interaction.local_spherical_3dof.aero_propulsive",
        "name": "interaction",
        "config": {}
      }
    ]
  }
}
```

`interaction` 只做闭合与组合，不拥有气动表、质量定义或推进模型。

## Vehicle Services

`vehicles[].services` 用于 vehicle scope 的基础设施服务。当前内置服务是 `coordinate_tree`：

```json
{
  "services": {
    "coordinate_tree": {
      "spec": "local_spherical_3dof.launch_track",
      "bindings": {
        "earth": { "name": "env.earth" },
        "truth": { "name": "dynamics" }
      },
      "launch": {
        "latitude_rad": 0.5235987755982988,
        "longitude_rad": 1.9198621771937625,
        "azimuth_rad": 1.5707963267948966,
        "launch_time_s": 0.0,
        "earth_rotation_angle_rad": 0.0
      }
    }
  }
}
```

服务先创建句柄，再在组件装配后 finalize，因此 service spec 可以绑定同一 vehicle 内的 form truth。项目组件通过 `injectServices(ServiceContext&)` 获取服务；示例见 `user/example_03_coordinate_tree/components/coordinate_probe.hpp`。

## Perturbation

`vehicles[].perturbation` 是可选的飞行器级组件块。它不属于
`common/input/process/output/interaction`，而是该 vehicle 的拉偏状态源。
省略时不会创建拉偏组件，普通单次 mission 的运行行为不变。

拉偏机制分三层：

1. **case 输入**：RunSet 的 `single/matrix/random` 或单次 mission 给出纯数字输入，例如温度档位、阻力偏置、推力倍率。
2. **拉偏组件解析**：`vehicles[].perturbation` 把这些数字解释成 number/string/vector 状态。字符串通常由整数输入通过 `enum_maps` 或项目自定义逻辑映射得到。
3. **其它组件读取**：气动、发动机、质量、制导等组件通过 `IPerturbationProvider` 读取解析后的状态，不直接读取 RunSet 配置。

RunSet 会在构建每个 case 的 effective mission 前，把该 case 的数值输入写入：

```text
vehicles[].perturbation.config.inputs
```

因此被 RunSet 引用的 vehicle 必须在 base mission 中已经声明
`perturbation` 块，RunSet 只替换其中的 `config.inputs`。effective mission 本身
就是完整的单 case 复现文件。复现时不需要再次读取 RunSet，也不依赖额外的
快照文件。

```json
{
  "perturbation": {
    "type": "perturbation.static",
    "name": "perturbation",
    "config": {
      "inputs": {
        "engine.temp_level": 2,
        "aero.drag_bias": -0.03
      },
      "enum_maps": {
        "engine.temp_level": {
          "0": "cold.txt",
          "2": "hot.txt"
        }
      }
    }
  }
}
```

`perturbation.static` exposes numeric inputs directly. When an `enum_maps` entry
matches an integer input, it also publishes a string value at
`<key>.resolved`, for example `engine.temp_level.resolved`.

上面的例子中：

- `engine.temp_level` 作为 number 输出，值为 `2`。
- `engine.temp_level.resolved` 作为 string 输出，值为 `"hot.txt"`。
- `aero.drag_bias` 作为 number 输出，值为 `-0.03`。

如果发动机组件要根据温度档位选择燃烧数据文件，它应在
`injectDependencies()` 中绑定 `IPerturbationProvider`，然后在 `initialize()`
读取 `engine.temp_level.resolved` 并加载相应资产。不要在 `configure()` 中读取
拉偏状态，因为组件配置阶段还没有完成组件间依赖注入。

静态拉偏和动态拉偏的边界如下：

| 类型 | 写在哪里 | 何时生效 |
| --- | --- | --- |
| 静态 case 输入 | RunSet case source 或单次 mission 的 `perturbation.config.inputs` | effective mission 构建完成后固定 |
| 静态解析状态 | 拉偏组件 `initialize()` 或 `configure()` 内部解析输入 | 其它组件 `initialize()` 后可读 |
| 动态拉偏逻辑 | 用户自定义拉偏组件的 `update()` | 每个周期在 `perturbation` stage 刷新 |

配置文件只描述静态 case 输入。随高度、马赫数、飞行阶段或其它状态变化的拉偏逻辑应写在自定义拉偏组件的 `update()` 中，而不是写成 RunSet 配置语法。由于 `perturbation` stage 位于 `environment` 之后、`vehicle.input/process/output/interaction` 之前，同一周期内后续组件可以读到本周期刷新的拉偏状态。

多飞行器任务中，每个 vehicle 都可以有自己的 `perturbation` 块。RunSet 的
`vehicles` 字段按 vehicle id 指定 case source；不同飞行器可以读取同一个矩阵文件，也可以读取不同文件或选用不同 rows。RunSet 要求所有参与的 vehicle 生成相同 case 数，按 case 序号配对运行。

## Component Scheduling

组件条目可以增加可选顶层字段 `priority` 和 `rate_hz`：

```json
{
  "type": "vehicle.process.programmed_aoa",
  "name": "guidance",
  "priority": 10,
  "rate_hz": 10.0,
  "config": {}
}
```

调度器先应用固定 stage 或 publish phase，再在同组内按 `priority` 排序；priority 相同保持 JSON 顺序。`priority` 必须是整数式 number，例如 `0`、`10` 或 `-5`；`0.5` 会作为 build error。默认 publish 序列中，priority 不会让 view 排到 state owner 前面。
`rate_hz` 必须 `> 0`，不能高于 `1 / simulation.dt`，并且必须让 `1 / simulation.dt / rate_hz` 形成整数步间隔；非法采样频率会作为 build error，不会隐式 round。

## Outputs

```json
{
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "cavh_3dof",
    "record_initial_state": true,
    "record": {
      "vehicle.dynamics": "all",
      "vehicle.guidance": "all"
    }
  }
}
```

`record_initial_state` 默认 `true`。CSV 每一行对应周期开始发布态 `t_k`；默认第一行是 `t0`。

CSV 行在 `update(t_k)` 之后写出：form/dynamics/truth view 字段是周期开始发布态 `x_k` 及其真实派生量；input/process/guidance/output 字段是组件在 record 时刻暴露的本周期离散输出。`t0` 行不是未经离散计算的纯初值行，而是初始状态 `x_0` 加上基于 `x_0` 计算出的第一周期离散输出。框架不保证所有 observable 在物理意义上无延迟；延迟语义由组件模型自身定义。`outputs` 顶层未知字段只产生 warning，不作为 build error。

`form.local_spherical_3dof.flight_state_view` 是 form/truth 层真实状态视图，在 publish 阶段刷新。机上导航或估计飞行状态应另建 `vehicle.process` 组件。

`record` 支持三种常用写法：

```json
{ "record": "all" }
```

```json
{ "record": ["vehicle.dynamics", "vehicle.guidance"] }
```

```json
{
  "record": {
    "vehicle.dynamics": ["altitude_m", "velocity_launch"],
    "vehicle.guidance": "all"
  }
}
```

字段数组按 observable 字段前缀匹配；`velocity_launch` 会记录 `velocity_launch.x/y/z`。`exclude` 可排除完整字段或通配后缀：

```json
{
  "exclude": [
    "vehicle.dynamics.velocity_ecef.x",
    "*.debug_counter"
  ]
}
```

`debug_snapshots` 记录组件内部 debug snapshot，不要求字段稳定，适合排查问题，不建议作为长期数据接口。

## Termination

```json
{
  "termination": {
      "type": "termination.component_field_below",
      "name": "termination",
      "config": {
      "component": "vehicle.dynamics",
      "field": "altitude_m",
      "value": 10000.0,
      "description": "Terminate below 10 km altitude"
    }
  }
}
```

`termination` 是顶层单例组件，必须实现 `ITerminationEvaluator`。内置阈值组件支持 `termination.component_field_below` 和 `termination.component_field_above`，并引用完整组件名和 observable/state 字段名。缺字段、类型错误、未知组件、未知字段和未知类型都会 build fail。旧 `stop_conditions[]` 不再支持。

## Summary

`summary` 是顶层单例组件，必须实现 `ISummaryObserver`。框架会保留基础 `summary.txt` 内容，并在其中追加 summary observer 写出的项目指标。
