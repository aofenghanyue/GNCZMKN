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
      "services": {},
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
    }
  ],
  "outputs": {},
  "stop_conditions": []
}
```

旧式 `entities[]`、根级 `components/services`、根级 `form/vehicle/interaction` 不属于当前 schema。

## Simulation

| 字段 | 规则 |
| --- | --- |
| `dt` | 必填，固定步长，必须 `> 0` |
| `duration` | 必填，必须 `>= 0` |
| `integrator` | 可省略，默认 `rk4`；当前支持 `rk4` 和 `euler` |

`duration / dt` 必须接近整数，否则 build 失败。未知 integrator 不回退，直接 build error。

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

## Component Priority

组件条目可以增加可选顶层字段 `priority`：

```json
{
  "type": "vehicle.process.programmed_aoa",
  "name": "guidance",
  "priority": 10,
  "config": {}
}
```

调度器先应用固定 stage 或 publish phase，再在同组内按 `priority` 排序；priority 相同保持 JSON 顺序。`priority` 必须是整数式 number，例如 `0`、`10` 或 `-5`；`0.5` 会作为 build error。默认 publish 序列中，priority 不会让 view 排到 state owner 前面。

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
## Stop Conditions

```json
{
  "stop_conditions": [
    {
      "type": "component_field_below",
      "component": "vehicle.dynamics",
      "field": "altitude_m",
      "value": 10000.0,
      "description": "Terminate below 10 km altitude"
    }
  ]
}
```

停止条件引用完整组件名和 observable/state 字段名。缺字段、类型错误、未知组件、未知字段和未知类型都会 build fail。
