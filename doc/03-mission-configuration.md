# 任务配置

任务配置使用 entity-first 结构。所有组件都属于某个实体，所有跨组件引用都应能追溯到明确的实体作用域。

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
| `simulation` | 是 | 仿真步长、时长、积分器、停止条件 |
| `outputs` | 否 | 自动记录器配置 |
| `global_services` | 否 | 全局服务配置 |
| `entities` | 是 | 环境实体和飞行器实体 |

`entities` 必须是数组。旧式的根级 `components`、`vehicles` 或 `services` 不作为推荐配置入口。

## simulation

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
| `integrator` | 当前支持 `rk4` 和 `euler`，未知值会回退到 `rk4` 并给出警告 |
| `stop_conditions` | 可选数组，按组件字段触发提前终止 |

停止条件当前支持 `component_field_below` 和 `component_field_above`。`component` 建议写完整组件名。

## outputs

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

`outputs.record` 有四种常用写法：

| 写法 | 含义 |
| --- | --- |
| 不写 `record` | 记录所有实现 `IObservable` 的字段 |
| `"record": "all"` | 记录所有实现 `IObservable` 的字段 |
| `"record": ["missile.dynamics"]` | 记录指定组件的全部可观测字段 |
| `"record": { "missile.dynamics": ["altitude_m"] }` | 记录指定字段或字段前缀 |

`outputs.exclude` 可以排除字段。`outputs.debug_snapshots` 可以记录组件主动写入的调试快照，适合临时排查，不应替代稳定的 `IObservable` 字段。

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
| `role` | 否 | 默认为 `vehicle`，环境实体写 `environment` |
| `services` | 否 | 当前实体的局部服务 |
| `components` | 是 | 当前实体内的组件列表 |

环境实体会先装配，飞行器实体后装配。当前只支持一个环境实体。

## 单飞行器任务

单飞行器任务仍然使用完整实体结构，不做特殊兼容分支。

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

装配后组件完整名为 `missile.guidance`、`missile.aero`、`missile.dynamics`、`env.earth`、`env.atmosphere`、`env.gravity`。

## 多飞行器任务

多飞行器任务只是增加多个 `vehicle` 实体。每个飞行器可以复用相同的局部组件名，因为实体前缀不同。

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

完整名分别是 `interceptor.dynamics` 和 `target.dynamics`。日志和停止条件必须写完整名。

## 服务配置

`soviet_coord` 是当前内置服务示例。它通常放在飞行器实体的 `services` 中，因为发射点和速度方向往往是飞行器相关配置。

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
- `outputs.record`、`simulation.stop_conditions`、服务 `bindings` 建议使用完整名。
- 组件代码里的同实体依赖可以使用局部名，例如 `"dynamics"`。
- 跨实体依赖应使用完整名，例如 `"env.atmosphere"`。

