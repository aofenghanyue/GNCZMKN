# 任务配置

任务配置采用 entity-first 结构。所有组件都属于某个实体，所有跨组件引用都应能追溯到明确的作用域。

## 顶层结构

```json
{
  "simulation": {},
  "outputs": {},
  "global_services": {},
  "entities": []
}
```

| 字段 | 必填 | 作用 |
| --- | --- | --- |
| `simulation` | 是 | 仿真步长、时长、积分器、停机条件 |
| `outputs` | 否 | 自动数据记录配置 |
| `global_services` | 否 | 全局服务配置 |
| `entities` | 是 | 环境实体和飞行器实体 |

注意：

- `entities` 必须是数组。
- 旧式根级 `components` / `services` / `vehicles` 已不再支持。
- 若继续使用旧写法，构建会直接失败，并提示迁移到 `entities[]`。

## `simulation`

```json
{
  "simulation": {
    "dt": 0.1,
    "duration": 600.0,
    "integrator": "rk4",
    "stop_conditions": [
      {
        "type": "component_field_below",
        "component": "missile.dynamics",
        "field": "altitude_m",
        "value": 10000.0,
        "description": "Terminate below 10 km altitude"
      }
    ]
  }
}
```

| 字段 | 说明 |
| --- | --- |
| `dt` | 固定仿真步长，单位秒 |
| `duration` | 最大仿真时长，单位秒 |
| `integrator` | 当前支持 `rk4` 和 `euler`。未知值会回退到 `rk4` 并给出警告 |
| `stop_conditions` | 可选数组，按组件字段提前终止仿真 |

当前停机条件支持：

- `component_field_below`
- `component_field_above`

建议始终在 `component` 字段里写完整组件名，例如 `missile.dynamics`。

## `outputs`

```json
{
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "atmospheric_3dof",
    "record": {
      "missile.dynamics": "all",
      "missile.guidance": "all",
      "missile.aero": ["lift_coefficient", "drag_coefficient"]
    }
  }
}
```

### 稳定字段与调试快照

- `outputs.record` 面向稳定输出。这里记录的字段通常会长期保留，并被日志、分析脚本和回归对比直接消费。
- `outputs.debug_snapshots` 面向临时排障。这里记录的是组件在某一步主动写入的调试值，不应替代稳定的 `IObservable` 字段。

换句话说：

- 要给用户长期看的量，放进 `IObservable`。
- 只为临时排查求值过程、残差、迭代状态的量，放进 `snapDebug()`。

### `record` 的 4 种常用写法

| 写法 | 含义 |
| --- | --- |
| 不写 `record` | 记录所有实现了 `IObservable` 的字段 |
| `"record": "all"` | 记录所有实现了 `IObservable` 的字段 |
| `"record": ["missile.dynamics", "missile.guidance"]` | 记录指定组件的全部可观察字段 |
| `"record": { "missile.dynamics": ["altitude_m", "velocity"], "missile.guidance": "all" }` | 按组件精确挑选字段或字段前缀 |

最后一种对象写法里：

- `"all"` 表示该组件的全部稳定字段。
- 数组里的每一项既可以是完整字段名，也可以是字段前缀。
- 例如 `"velocity"` 会匹配 `velocity.x`、`velocity.y`、`velocity.z`。

### `exclude`

`outputs.exclude` 只接受字符串数组，支持两种匹配语义：

| 写法 | 含义 |
| --- | --- |
| `"missile.dynamics.altitude_m"` | 排除完整字段名 |
| `"*.velocity.z"` | 排除所有以该后缀结尾的字段 |

示例：

```json
{
  "outputs": {
    "record": {
      "missile.dynamics": "all"
    },
    "exclude": [
      "missile.dynamics.altitude_m",
      "*.velocity.z"
    ]
  }
}
```

这个配置会保留 `missile.dynamics.velocity.x` 和 `missile.dynamics.velocity.y`，但排除：

- `missile.dynamics.altitude_m`
- 任意组件下名字以 `velocity.z` 结尾的字段

### `debug_snapshots`

`outputs.debug_snapshots` 支持两种写法。

布尔写法：

```json
{
  "outputs": {
    "debug_snapshots": true
  }
}
```

含义：

- `true`：记录所有组件主动写入的调试快照
- `false`：关闭调试快照

对象写法：

```json
{
  "outputs": {
    "session_name": "run_a",
    "debug_snapshots": {
      "components": ["missile.guidance"],
      "session_name": "guidance_debug",
      "precision": 6,
      "flush_every_step": true
    }
  }
}
```

对象字段说明：

| 字段 | 说明 |
| --- | --- |
| `enabled` | 是否启用。省略时默认 `true` |
| `components` | 要记录的组件过滤器。可省略、写 `"all"`、写单个字符串，或写字符串数组 |
| `session_name` | 调试快照文件名前缀。省略时默认使用 `<outputs.session_name>_debug_snapshots` |
| `precision` | 数值精度 |
| `flush_every_step` | 每一步后立即 flush 文件，适合边跑边盯日志 |

调试快照 CSV 的列固定为：

```text
time,component,field,value
```

### 一个能直接复制的 `outputs` 示例

```json
{
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "intercept_run",
    "record": {
      "missile.dynamics": "all",
      "missile.guidance": ["angle_of_attack_deg", "bank_angle_deg"],
      "missile.aero": ["lift", "drag"]
    },
    "exclude": [
      "*.velocity.z"
    ],
    "debug_snapshots": {
      "components": ["missile.guidance"],
      "session_name": "guidance_debug",
      "precision": 6,
      "flush_every_step": true
    }
  }
}
```

## Entity

实体配置格式：

```json
{
  "id": "missile",
  "role": "vehicle",
  "services": {},
  "components": []
}
```

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `id` | 是 | 实体标识。飞行器组件前缀来自这个字段 |
| `role` | 否 | 默认 `vehicle`。环境实体写 `environment` |
| `services` | 否 | 当前实体的局部服务 |
| `components` | 是 | 当前实体内的组件列表 |

环境实体会先装配，飞行器实体后装配。当前只支持一个环境实体。

## 单飞行器 mission

单飞行器 mission 也使用完整的实体结构，不再保留空前缀特例：

```json
{
  "entities": [
    {
      "id": "missile",
      "role": "vehicle",
      "components": [
        { "type": "example.programmed_aoa", "name": "guidance", "config": {} },
        { "type": "aero.simple_polynomial", "name": "aero", "config": {} },
        { "type": "state_3dof.point_mass_spherical", "name": "dynamics", "config": {} }
      ]
    },
    {
      "id": "environment",
      "role": "environment",
      "components": [
        { "type": "environment.wgs84_earth", "name": "earth", "config": {} },
        { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} },
        { "type": "environment.spherical_gravity", "name": "gravity", "config": {} }
      ]
    }
  ]
}
```

装配后完整名是：

- `missile.guidance`
- `missile.aero`
- `missile.dynamics`
- `env.earth`
- `env.atmosphere`
- `env.gravity`

## 多飞行器 mission

多飞行器 mission 只是增加多个 `vehicle` 实体。每个飞行器都可以复用相同的局部组件名，因为实体前缀不同。

```json
{
  "entities": [
    {
      "id": "interceptor",
      "role": "vehicle",
      "components": [
        { "type": "example.programmed_aoa", "name": "guidance", "config": {} },
        { "type": "state_3dof.point_mass_spherical", "name": "dynamics", "config": {} }
      ]
    },
    {
      "id": "target",
      "role": "vehicle",
      "components": [
        { "type": "state_3dof.point_mass_spherical", "name": "dynamics", "config": {} }
      ]
    }
  ]
}
```

对应完整名分别是：

- `interceptor.dynamics`
- `target.dynamics`

日志、停机条件和 `outputs.record` 都应写完整名。

## 服务配置

`soviet_coord` 是当前内置服务示例。它通常放在飞行器实体的 `services` 里，因为发射点和速度方向往往是飞行器局部配置。

```json
{
  "services": {
    "soviet_coord": {
      "launch": {
        "latitude_rad": 0.5235987755982988,
        "longitude_rad": 1.9198621771937625,
        "azimuth_rad": 1.5707963267948966,
        "launch_time_s": 0.0,
        "earth_rotation_angle_rad": 0.0
      },
      "bindings": {
        "earth": { "name": "env.earth" },
        "velocity_direction": { "name": "missile.dynamics" }
      }
    }
  }
}
```

服务安装可能产生延迟绑定动作。框架会先完成所有实体和组件注册，再执行这些动作，因此服务可以绑定后面才注册完成的组件。

## 命名规则

- 环境组件完整名固定为 `env.<component_name>`。
- 飞行器组件完整名固定为 `<entity_id>.<component_name>`。
- `outputs.record`、`simulation.stop_conditions` 和服务 `bindings` 建议始终使用完整名。
- 组件代码里的同实体依赖可以使用局部名，例如 `"dynamics"`。
- 跨实体依赖应使用完整名，例如 `"env.atmosphere"`。
> Archived note: this document uses the removed `entities[]` mission shape and
> plugin-era type ids. For the active mission schema, use
> [00-current-architecture.md](00-current-architecture.md).
