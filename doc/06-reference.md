# 参考手册

## Mission 顶层字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `simulation` | object | 必填；固定步长、时长、积分器 |
| `environment` | object | 可省略或为空；环境组件和服务 |
| `vehicles` | array | 必填；每项包含 `id/form/services/common/input/process/output/interaction` |
| `outputs` | object | 可省略；CSV 输出配置 |
| `stop_conditions` | array | 可省略；发布态停止条件 |

旧式 `entities[]`、根级 `components/services`、根级 `form/vehicle/interaction` 不支持。

## Simulation 字段

| 字段 | 规则 |
| --- | --- |
| `dt` | 必填 number，`> 0` |
| `duration` | 必填 number，`>= 0` |
| `integrator` | string，`rk4` 或 `euler`，默认 `rk4` |

`duration / dt` 必须是整数步数。

## Component Entry 字段

每个组件条目支持 `type`、`name`、可选 `priority` 和 `config`。
`priority` 是整数式 number。同一 runtime stage 或 publish phase 内，较小 priority 先执行；priority 相同保持 JSON 顺序。小数 priority 会作为 build error。默认 publish phase 顺序仍是 state owner、view / observer、普通组件。

## Builtin 组件

| Type id | Mission 位置 | 主要接口 |
| --- | --- | --- |
| `form.cartesian_3dof.point_mass` | `vehicles[].form.components` | `IContinuousSystem`, Cartesian `ITruthView`, `IObservable` |
| `form.local_spherical_3dof.point_mass` | `vehicles[].form.components` | `IContinuousSystem`, local-spherical `ITruthView`, `IObservable` |
| `form.local_spherical_3dof.flight_state_view` | `vehicles[].form.components` | truth/form `IFlightStateView`, `IObservable` |
| `interaction.cartesian_3dof.direct_accel` | `vehicles[].interaction.components` | Cartesian `IInputProvider` |
| `interaction.local_spherical_3dof.direct_accel` | `vehicles[].interaction.components` | local-spherical `IInputProvider` |
| `interaction.local_spherical_3dof.aero_propulsive` | `vehicles[].interaction.components` | local-spherical `IInputProvider` |
| `vehicle.process.programmed_aoa` | `vehicles[].process` | `IAeroGuidanceProvider`, `IObservable` |
| `aero.simple_polynomial` | `vehicles[].output` | `IAeroModel`, `IObservable` |
| `aero.table2d` | `vehicles[].output` | `IAeroModel`, `IObservable` |
| `mass.constant` | `vehicles[].output` | `IConstantMass`, `IObservable` |
| `mass.continuous_constant_rate` | `vehicles[].output` | `IContinuousMass`, `IContinuousSystem`, `IObservable` |

## 常用配置字段

`form.cartesian_3dof.point_mass`:

| 字段 | 规则 |
| --- | --- |
| `initial_position` | 必填，3 个 number |
| `initial_velocity` | 必填，3 个 number |
| `input_lookup_name` | 可选，默认 `interaction` |

`form.local_spherical_3dof.point_mass`:

| 字段 | 规则 |
| --- | --- |
| `launch_azimuth_rad` | 必填 number |
| `initial_state.longitude_rad` | 必填 number |
| `initial_state.latitude_rad` | 必填 number |
| `initial_state.altitude_m` | 必填 number |
| `initial_state.speed_mps` | 必填 number |
| `initial_state.flight_path_angle_rad` | 必填 number |
| `initial_state.heading_angle_rad` | 必填 number |
| `earth_lookup_name` | 可选，默认 `env.earth` |
| `input_lookup_name` | 可选，默认 `interaction` |

`mass.constant` 要求 `mass_kg`，可来自 inline config 或 JSON asset。

`aero.table2d` 要求 `reference_area_m2`、`reference_length_m`、`alpha_breaks_rad`、`mach_breaks`、`lift_coefficients`、`drag_coefficients`。

## Outputs

| 字段 | 默认 | 说明 |
| --- | --- | --- |
| `enabled` | `true` | `false` 时关闭自动记录 |
| `directory` | `user/outputs` | 支持 `{timestamp}` |
| `format` | `csv` | 当前仅支持 CSV |
| `session_name` | `simulation_data` | 输出文件名前缀 |
| `record_initial_state` | `true` | 是否记录 t0 发布态 |
| `record` | `all` | 记录规则 |
| `exclude` | `[]` | 排除完整字段或 `*.suffix` |
| `debug_snapshots` | disabled | 调试快照 |

CSV `time` 列表示发布态物理时间。

CSV 行在 `update(t_k)` 之后写出：form/dynamics/truth view 字段是周期开始发布态 `x_k` 及其真实派生量；input/process/guidance/output 字段是 record 时刻组件暴露的本周期离散输出。`t0` 行包含初始状态 `x_0` 和基于 `x_0` 计算出的第一周期离散输出。框架不保证所有 observable 在物理意义上无延迟；延迟语义由组件模型自身定义。`outputs` 顶层未知字段只产生 build warning，不作为 build error。
## Stop Conditions

支持：

- `component_field_below`
- `component_field_above`

字段：

| 字段 | 规则 |
| --- | --- |
| `type` | 必填 string |
| `component` | 必填完整组件名 |
| `field` | 必填 observable/state 字段 |
| `value` | 必填 number |
| `description` | 可选 string |
