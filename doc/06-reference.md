# 参考手册

这篇文档用于查细节，不建议作为第一篇阅读材料。配置写法的解释以 [任务配置](03-mission-configuration.md) 为准，这里只保留速查表。

## 命令行

| 命令 | 作用 |
| --- | --- |
| `gnc_sim.exe` | 运行活动项目默认 mission |
| `gnc_sim.exe <config.json>` | 运行指定 mission |
| `gnc_sim.exe --config <config.json>` | 运行指定 mission |
| `gnc_sim.exe --list-components` | 列出已注册组件 type id |
| `gnc_sim.exe --list-components-verbose` | 列出 type id、接口列表和注册来源 |
| `gnc_sim.exe --help` | 打印帮助 |

任务路径可以是绝对路径、当前目录相对路径或仓库相对路径。runner 会从当前目录和可执行文件目录向上搜索。

## CMake 选项

| 选项 | 默认值 | 说明 |
| --- | --- | --- |
| `BUILD_TESTS` | `OFF` | 是否构建测试 |
| `GNC_ACTIVE_PROJECT` | 空 | 指定 `user/` 下的活动项目名 |

若未显式设置 `GNC_ACTIVE_PROJECT`，CMake 会读取 `user/active_project`。

## Mission 字段

| 路径 | 类型 | 说明 |
| --- | --- | --- |
| `simulation.dt` | number | 固定步长，单位秒 |
| `simulation.duration` | number | 最大仿真时长，单位秒 |
| `simulation.integrator` | string | `rk4` 或 `euler` |
| `simulation.stop_conditions` | array | 停机条件 |
| `outputs.enabled` | bool | 是否启用自动记录器 |
| `outputs.directory` | string | 输出目录，支持 `{timestamp}` |
| `outputs.format` | string | 当前只支持 `csv` |
| `outputs.session_name` | string | 输出文件名前缀 |
| `outputs.record` | string / array / object | 稳定字段记录规则 |
| `outputs.exclude` | array | 排除字段。支持完整字段名和 `*.suffix` |
| `outputs.debug_snapshots` | bool / object | 调试快照配置。对象写法支持 `components`、`session_name`、`precision`、`flush_every_step` |
| `global_services` | object | 全局服务 |
| `entities` | array | 实体列表 |

补充约束：

- 顶层必须使用 `entities[]`
- 旧式根级 `components` / `services` / `vehicles` 已不支持

## Entity 字段

| 路径 | 类型 | 说明 |
| --- | --- | --- |
| `entities[].id` | string | 实体标识 |
| `entities[].role` | string | `environment` 或 `vehicle`，默认 `vehicle` |
| `entities[].services` | object | 实体局部服务 |
| `entities[].components` | array | 实体组件 |
| `components[].type` | string | 组件 type id |
| `components[].name` | string | 实体内局部组件名 |
| `components[].config` | object | 组件配置 |

## 内置组件

| type id | 插件 | 主要接口 |
| --- | --- | --- |
| `environment.wgs84_earth` | `environment` | `IEarth` |
| `environment.standard_atmosphere` | `environment` | `IAtmosphere` |
| `environment.spherical_gravity` | `environment` | `IGravity` |
| `aero.simple_polynomial` | `aero` | `IAeroModel`, `IObservable` |
| `state_3dof.point_mass_cartesian` | `state_3dof` | `IContinuousSystem`, `IStateSolver3DOF`, `IObservable` |
| `state_3dof.point_mass_spherical` | `state_3dof` | `IContinuousSystem`, `IStateSolver3DOF`, `IVelocityDirectionProvider`, `IObservable` |

项目示例组件：

| type id | 位置 | 作用 |
| --- | --- | --- |
| `example.programmed_aoa` | `user/example_02_atmospheric_3dof/components/` | 按高度表输出攻角指令 |
| `example.coordinate_probe` | `user/example_03_soviet_coord/components/` | 调用坐标服务并输出探针结果 |

## 内置服务

| 服务名 | 插件 | 作用 |
| --- | --- | --- |
| `soviet_coord` | `soviet_coord` | 安装 `ISovietCoordService`，提供坐标系转换 |

`soviet_coord` 配置字段：

| 路径 | 说明 |
| --- | --- |
| `launch.latitude_rad` | 发射点纬度 |
| `launch.longitude_rad` | 发射点经度 |
| `launch.azimuth_rad` | 发射方位角 |
| `launch.launch_time_s` | 发射时间 |
| `launch.earth_rotation_angle_rad` | 初始地球自转角 |
| `bindings.earth.name` | 实现 `IEarth` 的组件 |
| `bindings.velocity_direction.name` | 可选，速度方向提供者 |
| `bindings.body_attitude.name` | 可选，体轴姿态提供者 |
| `bindings.body_airspeed.name` | 可选，体轴空速提供者 |

## 常用组件配置

`aero.simple_polynomial`

| 字段 | 说明 |
| --- | --- |
| `lift_offset` | 升力系数常量项 |
| `lift_slope_per_rad` | 升力系数对攻角的斜率 |
| `drag_zero` | 零攻角阻力系数 |
| `drag_quadratic` | 阻力系数二次项 |
| `reference_area_m2` | 参考面积 |
| `reference_length_m` | 参考长度 |

`state_3dof.point_mass_spherical`

| 字段 | 说明 |
| --- | --- |
| `launch_azimuth_rad` | 发射方位角 |
| `mass_kg` | 质量 |
| `reference_radius_m` | 无 `IEarth` 时使用的参考半径 |
| `initial_state.longitude_rad` | 初始经度 |
| `initial_state.latitude_rad` | 初始纬度 |
| `initial_state.altitude_m` | 初始高度 |
| `initial_state.speed_mps` | 初始速度 |
| `initial_state.flight_path_angle_rad` | 初始航迹倾角 |
| `initial_state.heading_angle_rad` | 初始航向角 |

`state_3dof.point_mass_cartesian`

| 字段 | 说明 |
| --- | --- |
| `initial_position` | 三元数组，初始位置 |
| `initial_velocity` | 三元数组，初始速度 |
| `constant_acceleration` | 三元数组，常加速度 |
| `mass_kg` | 质量 |

## 命名规则

| 对象 | 规则 | 示例 |
| --- | --- | --- |
| 环境组件完整名 | `env.<component_name>` | `env.atmosphere` |
| 飞行器组件完整名 | `<entity_id>.<component_name>` | `missile.dynamics` |
| 同实体依赖 | 可在组件代码里写局部名 | `dynamics` |
| 跨实体依赖 | 使用完整名 | `env.gravity` |
| 日志和停机条件 | 使用完整名 | `missile.dynamics.altitude_m` |
> Archived note: entries in this file include removed plugin-era type ids and
> service names. Do not use them for new code or mission authoring; use
> [00-current-architecture.md](00-current-architecture.md) instead.
