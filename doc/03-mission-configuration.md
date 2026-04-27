# Mission 配置

mission 是 GNCZMKN 的装配入口。它声明仿真参数、form、环境、飞行器组件、interaction、输出和停止条件。

## 顶层结构

当前有效结构是：

```json
{
  "simulation": {},
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle_id",
      "form": {},
      "services": {},
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {}
    }
  ],
  "outputs": {},
  "stop_conditions": [],
  "global_services": {}
}
```

`stop_conditions` 和 `global_services` 可按需省略。旧式 `entities[]`、根级 `components/services`，以及根级 `form`、`vehicle`、`interaction` 不属于当前 schema。mission 必须使用顶层 `vehicles` 数组。单飞行器任务就是 `vehicles` 中只有一个条目。

每个 `vehicles[]` 条目代表一个自成一体的广义飞行器实体，可以是飞行器、目标点或其他拥有自身状态和能力边界的对象。该条目的 `form`、`services`、`common/input/process/output` 和 `interaction` 共用同一个作用域，组件全名使用 `<id>.<name>`：

```json
{
  "simulation": {},
  "environment": {},
  "vehicles": [
    {
      "id": "chaser",
      "form": {
        "components": [
          { "type": "form.cartesian_3dof.point_mass", "name": "dynamics", "config": {} }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          { "type": "interaction.cartesian_3dof.direct_accel", "name": "interaction", "config": {} }
        ]
      }
    },
    {
      "id": "target",
      "form": {
        "components": [
          { "type": "form.cartesian_3dof.point_mass", "name": "dynamics", "config": {} }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          { "type": "interaction.cartesian_3dof.direct_accel", "name": "interaction", "config": {} }
        ]
      }
    }
  ],
  "outputs": {
    "record": {
      "chaser.dynamics": "all",
      "target.dynamics": "all"
    }
  }
}
```

`id` 必须是 `[A-Za-z_][A-Za-z0-9_-]*`，且不能是 `env`。同一飞行器内部依赖仍可写 local name，例如 `dynamics` 或 `interaction` 会在该飞行器作用域内解析；跨环境依赖继续写完整名，例如 `env.earth`。form-family 校验按 vehicle 作用域执行，因此不同飞行器可以选择不同 form family。

## Simulation

`simulation` 控制固定步长运行：

```json
{
  "simulation": {
    "dt": 0.1,
    "duration": 600.0,
    "integrator": "rk4"
  }
}
```

常用字段：

| 字段 | 含义 |
| --- | --- |
| `dt` | 固定步长，单位秒 |
| `duration` | 最大仿真时长，单位秒 |
| `integrator` | 当前常用 `rk4` 或 `euler` |

## Form

`form.components` 声明被积分状态和 form-local view。例如 local-spherical 3DoF：

```json
{
  "id": "cavh",
  "form": {
    "components": [
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
      },
      {
        "type": "form.local_spherical_3dof.flight_state_view",
        "name": "flight_state",
        "config": {}
      }
    ]
  }
}
```

这两个组件的 registry 全名分别是 `cavh.dynamics` 和 `cavh.flight_state`。

## Environment

`environment.components` 声明环境查询能力：

```json
{
  "environment": {
    "components": [
      { "type": "environment.spherical_earth", "name": "earth", "config": {} },
      { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} },
      { "type": "environment.spherical_gravity", "name": "gravity", "config": {} }
    ]
  }
}
```

这些组件的全名是 `env.earth`、`env.atmosphere` 和 `env.gravity`。

## Vehicle

`vehicle` 是飞行器侧组件和服务的容器：

```json
{
  "id": "cavh",
  "common": [],
  "input": [],
  "process": [],
  "output": []
}
```

职责划分：

| 块 | 放什么 |
| --- | --- |
| `common` | 静态资产/profile/参数包，不参与 stage 调度 |
| `input` | 传感器和测量侧硬件 |
| `process` | 导航、制导、控制、命令生成 |
| `output` | 气动、质量、推进、分离等运行时物理效应 |

当前 atmospheric 示例使用：

```json
{
  "id": "cavh",
  "common": [],
  "input": [],
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
  ]
}
```

## Interaction

`interaction.components` 声明 form-aware closure：

```json
{
  "id": "cavh",
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

这个组件读取 local-spherical truth、环境能力、同作用域内的 `guidance` 命令、`mass` 和 `aero` 等 output 能力，然后写入 form input。

## Outputs

`outputs` 控制自动记录：

```json
{
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "cavh_3dof",
    "record": {
      "cavh.dynamics": "all",
      "cavh.guidance": "all",
      "cavh.aero": "all",
      "cavh.mass": "all",
      "cavh.flight_state": "all"
    }
  }
}
```

常用字段：

| 字段 | 含义 |
| --- | --- |
| `enabled` | `false` 时关闭自动记录 |
| `directory` | 输出目录，支持 `{timestamp}` |
| `format` | 当前使用 `csv` |
| `session_name` | 输出文件名前缀 |
| `record` | 记录规则，可以按组件记录 `all` 或字段列表 |
| `exclude` | 排除完整字段名或 `*.suffix` 后缀匹配 |
| `debug_snapshots` | 调试快照记录，可为 bool 或配置对象 |

只有实现 `IObservable` 的组件会提供 stable observable 字段。

## Stop Conditions

`stop_conditions` 是顶层数组：

```json
{
  "stop_conditions": [
    {
      "type": "component_field_below",
      "component": "cavh.dynamics",
      "field": "altitude_m",
      "value": 10000.0,
      "description": "Terminate below 10 km altitude"
    }
  ]
}
```

停止条件使用完整组件名和 observable 字段名。Cartesian 示例中的高度字段是 `altitude`；local-spherical 示例中的高度字段是 `altitude_m`。

## Coordinate Tree 服务

`coordinate_tree` 当前 v1 只支持 `vehicles[].services.coordinate_tree`：

```json
{
  "id": "cavh",
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

`global_services.coordinate_tree` 和 `environment.services.coordinate_tree` 会被拒绝。当前支持的 spec 见 [参考手册](06-reference.md)。

## 示例选择

| 示例 | 用途 |
| --- | --- |
| `user/example_01_minimal_pluginized/config/mission.json` | 最小 Cartesian 3DoF 示例，目录名保留历史痕迹 |
| `user/example_02_atmospheric_3dof/config/mission.json` | 推荐主示例，覆盖 environment、process、output、interaction 和输出记录 |
| `user/example_03_coordinate_tree/config/mission.json` | coordinate-tree 服务和项目组件示例 |
