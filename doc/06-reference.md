# 参考手册

这篇文档用于快速查命令、mission 字段、builtin type id、服务和命名规则。概念解释请看 [核心概念](02-core-concepts.md)，配置示例请看 [Mission 配置](03-mission-configuration.md)。

## 命令行

| 命令 | 用途 |
| --- | --- |
| `gnc_sim.exe` | 运行 active project 的默认 mission |
| `gnc_sim.exe <config.json>` | 运行指定 mission |
| `gnc_sim.exe --config <config.json>` | 运行指定 mission |
| `gnc_sim.exe --list-components` | 列出注册组件 type id |
| `gnc_sim.exe --list-components-verbose` | 列出 type id、接口、role、stage、form family 和注册来源 |
| `gnc_sim.exe --help` | 打印命令帮助 |

路径可以是绝对路径、当前目录相对路径或仓库根相对路径。runner 会从当前工作目录和可执行文件目录向上搜索。

## Mission 顶层块

| Path | Type | 用途 |
| --- | --- | --- |
| `simulation` | object | 步长、时长、积分器 |
| `form` | object | 状态和 form-local 组件 |
| `environment` | object | 环境组件和环境服务 |
| `vehicle` | object | vehicle 服务和 `common/input/process/output` 组件 |
| `interaction` | object | 从 environment/process/output 到 form input 的闭合层 |
| `outputs` | object | 自动记录配置 |
| `stop_conditions` | array | 停止条件 |
| `global_services` | object | 全局服务配置，当前很少使用 |

旧式 `entities[]`、根级 `components`、根级 `services` 和根级 `vehicles` 不支持。

## 常用 Mission 字段

| Path | Type | 用途 |
| --- | --- | --- |
| `simulation.dt` | number | 固定步长，单位秒 |
| `simulation.duration` | number | 最大仿真时长，单位秒 |
| `simulation.integrator` | string | `rk4` 或 `euler` |
| `outputs.enabled` | bool | 是否启用自动记录 |
| `outputs.directory` | string | 输出目录，支持 `{timestamp}` |
| `outputs.format` | string | 当前使用 `csv` |
| `outputs.session_name` | string | 输出文件名前缀 |
| `outputs.record` | string/array/object | stable observable 字段记录规则 |
| `outputs.exclude` | array | 完整字段名或 `*.suffix` 排除规则 |
| `outputs.debug_snapshots` | bool/object | debug snapshot 记录规则 |

## Builtin Components

| Type id | 放置位置 | 主要接口 |
| --- | --- | --- |
| `environment.spherical_earth` | `environment.components` | `IEarth` |
| `environment.wgs84_earth` | `environment.components` | `IEarth` |
| `environment.standard_atmosphere` | `environment.components` | `IAtmosphere` |
| `environment.spherical_gravity` | `environment.components` | `IGravity` |
| `form.cartesian_3dof.point_mass` | `form.components` | `IContinuousSystem`, `ITruthView`, `IObservable` |
| `form.local_spherical_3dof.point_mass` | `form.components` | `IContinuousSystem`, `ITruthView`, `IObservable` |
| `form.local_spherical_3dof.flight_state_view` | `form.components` | `IFlightStateView`, `IObservable` |
| `interaction.cartesian_3dof.direct_accel` | `interaction.components` | Cartesian `IInputProvider` |
| `interaction.local_spherical_3dof.direct_accel` | `interaction.components` | local-spherical `IInputProvider` |
| `interaction.local_spherical_3dof.aero_propulsive` | `interaction.components` | local-spherical `IInputProvider` |
| `vehicle.process.programmed_aoa` | `vehicle.process` | `IAeroGuidanceProvider`, `IObservable` |
| `aero.simple_polynomial` | `vehicle.output` | `IAeroModel`, `IObservable` |
| `aero.table2d` | `vehicle.output` | `IAeroModel`, `IObservable` |
| `mass.constant` | `vehicle.output` | `IConstantMass`, `IObservable` |
| `mass.continuous_constant_rate` | `vehicle.output` | `IContinuousMass`, `IContinuousSystem`, `IObservable` |

示例项目组件不是稳定 builtin API。例如 `example.coordinate_probe` 位于 `user/example_03_coordinate_tree/components/`，只有该项目作为 active project 并重新构建后才会自动注册。

## Role 和 Stage 标签

`--list-components-verbose` 中常见标签：

| 标签 | 含义 |
| --- | --- |
| `environment` | 环境组件和环境 stage |
| `form` | form 组件和 form stage |
| `vehicle_common` | vehicle common，非调度 |
| `vehicle_input` | vehicle input stage |
| `vehicle_process` | vehicle process stage |
| `vehicle_output` | vehicle output stage |
| `interaction` | interaction stage |
| `none` | 不参与 stage 调度 |

## Builtin Services

| Service id | 支持位置 | 用途 |
| --- | --- | --- |
| `coordinate_tree` | `vehicle.services.coordinate_tree` | 安装 `ICoordService`，提供坐标系转换 |

当前 `coordinate_tree` v1 只支持 vehicle scope。放在 `global_services.coordinate_tree` 或 `environment.services.coordinate_tree` 会失败。

## Coordinate-tree Specs

| Spec id | 用途 |
| --- | --- |
| `empty` | 只创建 root frame `I` |
| `local_spherical_3dof.launch_track` | 构建当前 local-spherical launch/track frame tree |

`local_spherical_3dof.launch_track` 常用字段：

| Path | 用途 |
| --- | --- |
| `spec` | 必须是 `local_spherical_3dof.launch_track` |
| `bindings.earth.name` | 实现 `IEarth` 的组件名，通常是 `env.earth` |
| `bindings.truth.name` | local-spherical `ITruthView` 组件名，vehicle scope 内可写 `dynamics` |
| `launch.latitude_rad` | 发射纬度 |
| `launch.longitude_rad` | 发射经度 |
| `launch.azimuth_rad` | 发射方位角 |
| `launch.launch_time_s` | 发射时间 |
| `launch.earth_rotation_angle_rad` | 初始地球自转角 |

## 命名规则

| 对象 | 规则 | 示例 |
| --- | --- | --- |
| 环境组件全名 | `env.<name>` | `env.atmosphere` |
| Form 组件全名 | `vehicle.<name>` | `vehicle.dynamics` |
| Vehicle 组件全名 | `vehicle.<name>` | `vehicle.guidance` |
| Interaction 组件全名 | `vehicle.<name>` | `vehicle.interaction` |
| 同 vehicle 依赖查询 | local name 或 full name | `dynamics` 或 `vehicle.dynamics` |
| 跨环境依赖查询 | full name | `env.gravity` |
| 输出和停止条件 | full name + field | `vehicle.dynamics.altitude_m` |

## 输出记录规则

记录一个组件全部字段：

```json
{
  "record": {
    "vehicle.dynamics": "all"
  }
}
```

记录字段列表：

```json
{
  "record": {
    "vehicle.dynamics": ["altitude_m", "speed_mps"]
  }
}
```

排除字段：

```json
{
  "exclude": [
    "vehicle.dynamics.altitude_m",
    "*.velocity.z"
  ]
}
```

关闭输出：

```json
{
  "outputs": {
    "enabled": false
  }
}
```

## 示例 Mission

| 路径 | 用途 |
| --- | --- |
| `user/config/missions/default.json` | repository fallback mission |
| `user/example_01_minimal_pluginized/config/mission.json` | 最小 Cartesian 3DoF 示例，目录名保留历史痕迹 |
| `user/example_02_atmospheric_3dof/config/mission.json` | 推荐 atmospheric local-spherical 3DoF 示例 |
| `user/example_03_coordinate_tree/config/mission.json` | coordinate-tree 服务示例 |
