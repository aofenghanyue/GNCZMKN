# 参考手册

## CLI

| 命令 | 说明 |
| --- | --- |
| `gnc_sim.exe` | 运行构建时 active project 的默认 mission |
| `gnc_sim.exe <config.json>` | 运行指定 mission |
| `gnc_sim.exe --config <config.json>` | 运行指定 mission |
| `gnc_sim.exe --simflow <simflow.json>` | 串行运行 SimFlow cases |
| `gnc_sim.exe --simflow <simflow.json> --jobs 1` | 显式串行运行 SimFlow cases |
| `gnc_sim.exe --simflow <simflow.json> --jobs auto` | 多进程运行；并发数为 `max(1, hardware_concurrency - 1)` |
| `gnc_sim.exe --simflow <simflow.json> --jobs <N>` | 多进程运行；最多同时运行 `N` 个 case |
| `gnc_sim.exe --list-components` | 列出已注册 type id |
| `gnc_sim.exe --list-components-verbose` | 列出 type id、接口、role、stage、form family 和注册来源 |
| `gnc_sim.exe --help` | 打印帮助 |

相对 mission 路径会从当前目录和可执行文件目录向上搜索。默认 mission 来自构建时 active project 的 `user/<project>/config/mission.json`。

SimFlow paths are loaded as provided. A SimFlow references a base mission,
uses a materializer to generate one effective mission per case, writes case
reproduction files, and then runs each case through the normal
`SimulationBuilder` / `Simulator` path.

## 源码入口速查

| 入口 | 用途 |
| --- | --- |
| `src/runner.cpp` | CLI、默认 mission 查找、注册 builtin/project type、启动 build/run |
| `framework/include/gnc/core/simulation_builder.hpp` | mission build 协调、simulation 校验、integrator、logger 初始化 |
| `framework/include/gnc/core/mission_assembler.hpp` | environment、vehicles、services、components、termination、summary 装配 |
| `framework/include/gnc/core/simulator.hpp` | publish、stage update、record、termination、integration、finalize |
| `framework/include/gnc/core/component_factory.hpp` | type id 注册、接口元数据、role/stage/form family 元数据 |
| `framework/include/gnc/simflow/simflow_runner.hpp` | SimFlow config loading, case materialization, effective mission writing, serial/multiprocess case execution |
| `framework/include/gnc/core/scoped_registry.hpp` | 当前 scope 内短名依赖解析和接口绑定 |
| `framework/include/gnc/infrastructure/auto_data_logger.hpp` | `outputs` 配置、observable 字段发现、CSV/debug snapshot 输出 |

## 修改类型速查

| 目标 | 首选文档 | 关键源码 |
| --- | --- | --- |
| 写 mission | [03-mission-configuration.md](03-mission-configuration.md) | `MissionAssembler` |
| 新增项目组件 | [04-extension-guide.md](04-extension-guide.md) | active project `components/` 和 `component_factory.hpp` |
| 新增 builtin 组件 | [04-extension-guide.md](04-extension-guide.md) | `register_builtin_packages.hpp` |
| 理解运行时顺序 | [00-current-architecture.md](00-current-architecture.md), [05-architecture.md](05-architecture.md) | `simulator.hpp` |
| 调整输出记录 | [03-mission-configuration.md](03-mission-configuration.md) | `auto_data_logger.hpp` |
| 调整 service | [05-architecture.md](05-architecture.md) | `service_package_registry.hpp` 和对应 `services/*/bootstrap` |

## Mission 顶层字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `simulation` | object | 必填；固定步长、时长、积分器 |
| `global_services` | object | 可省略；高级顶层服务块，当前内置服务通常不支持 global scope |
| `environment` | object | 可省略或为空；环境组件和服务 |
| `vehicles` | array | 必填；每项包含 `id/form/services/common/input/process/output/interaction` |
| `outputs` | object | 可省略；CSV 输出配置 |
| `termination` | object | 可省略；顶层 termination 组件 |
| `summary` | object | 可省略；顶层 summary observer 组件 |

旧式 `entities[]`、根级 `components/services`、根级 `form/vehicle/interaction` 不支持。

Each `vehicles[]` entry may also include optional `perturbation`. When present,
it is a single component entry at vehicle scope and must implement
`IPerturbationProvider`.

## Include Roots

| Prefix | 解析位置 |
| --- | --- |
| relative path | 包含 `$include` 的 JSON 文件所在目录 |
| `repo://` | 仓库根目录 |
| `project://` | 当前 `user/<project>/` 根目录 |
| `user-data://` | `user/data/` |

`$include` 可以是 string 或有序 string array。多个 include 按顺序 deep merge，本地字段覆盖 include 字段，数组整体替换。

## Simulation 字段

| 字段 | 规则 |
| --- | --- |
| `dt` | 必填 number，`> 0` |
| `duration` | 必填 number，`>= 0` |
| `integrator` | string，`rk4` 或 `euler`，默认 `rk4` |

`duration / dt` 必须是整数步数。

## Component Entry 字段

每个组件条目支持 `type`、`name`、可选 `priority`、`rate_hz` 和 `config`。

| 字段 | 规则 |
| --- | --- |
| `type` | 必填 string；必须是已注册 type id |
| `name` | 必填 string；与作用域前缀组成完整组件名 |
| `priority` | 可选整数式 number；同组内越小越先执行 |
| `rate_hz` | 可选 number；`> 0`，不高于仿真频率，且形成整数步间隔 |
| `config` | object；传给组件自己的 `configure()` |

`priority` 是整数式 number。同一 runtime stage 或 publish phase 内，较小 priority 先执行；priority 相同保持 JSON 顺序。小数 priority 会作为 build error。默认 publish phase 顺序仍是 state owner、view / observer、普通组件。`rate_hz` 必须 `> 0`、不高于 `1 / simulation.dt`，且必须形成整数步间隔。

## Builtin 组件

| Type id | Mission 位置 | 主要接口 |
| --- | --- | --- |
| `environment.spherical_earth` | `environment.components` | `IEarth` |
| `environment.wgs84_earth` | `environment.components` | `IEarth` |
| `environment.standard_atmosphere` | `environment.components` | `IAtmosphere` |
| `environment.spherical_gravity` | `environment.components` | `IGravity` |
| `form.cartesian_3dof.point_mass` | `vehicles[].form.components` | `IContinuousSystem`, Cartesian `ITruthView`, `IObservable` |
| `form.local_spherical_3dof.point_mass` | `vehicles[].form.components` | `IContinuousSystem`, local-spherical `ITruthView`, `IObservable` |
| `perturbation.static` | `vehicles[].perturbation` | `IPerturbationProvider`, `IPerturbationSnapshot` |
| `termination.component_field_below` | `termination` | `ITerminationEvaluator` |
| `termination.component_field_above` | `termination` | `ITerminationEvaluator` |

## Builtin 服务

| Service id | Mission 位置 | 说明 |
| --- | --- | --- |
| `coordinate_tree` | `vehicles[].services.coordinate_tree` | 构建 vehicle-scoped 坐标树服务，组件通过 `ICoordService` 查询 |

`coordinate_tree` 当前支持 specs：

| Spec id | 说明 |
| --- | --- |
| `empty` | 只创建根 frame `I` |
| `local_spherical_3dof.launch_track` | 基于 local-spherical truth、地球模型和发射参数创建 `I/E/N/L/LI/K` 等 frame |

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

`environment.spherical_earth`:

| 字段 | 规则 |
| --- | --- |
| `equatorial_radius_m` | 可选 number，默认 `6371000.0` |
| `rotation_rate_rad_per_s` | 可选 number，默认 `7.292115e-5` |

`environment.wgs84_earth` 和 `environment.standard_atmosphere` 不需要配置字段。`environment.spherical_gravity` 支持可选 `reference_radius_m` 和 `sea_level_gravity_mps2`。

`perturbation.static`:

| 字段 | 规则 |
| --- | --- |
| `inputs` | 可选 object；每个 value 必须是 number |
| `enum_maps` | 可选 object；将整数 numeric input 映射为 string resolved value |

For each numeric input, `perturbation.static` exposes the same key as a number.
For each matching enum map, it exposes `<key>.resolved` as a string.

`vehicle.output.mass_3dof.constant` requires `mass_kg`.
`vehicle.output.mass_properties_6dof.constant` requires `mass_kg`, `center_of_gravity_body_m`, and `inertia_body_kgm2`.
`interaction.cartesian_3dof.standard` accepts optional `acceleration_mps2` and binds the normal 3DOF output chain.
`interaction.local_spherical_3dof.standard` accepts optional `local_acceleration_nue_mps2` and binds the normal 3DOF output chain.
Task-specific aerodynamic tables, guidance laws, and closure models belong in `user/<project>/components`; see `user/example_08_cavh_geographic_3dof_custom`.

## SimFlow

SimFlow config is a separate JSON file for pre-simulation materialization and batch execution:

This section is a field reference. For the mechanism and component dependency
model, read [03-mission-configuration.md](03-mission-configuration.md) and
[04-extension-guide.md](04-extension-guide.md).

```json
{
  "base_mission": "user/example/config/mission.json",
  "materializer": {
    "type": "simflow.materializer.numeric_perturbation",
    "config": {
      "case_source": {
        "mode": "matrix",
        "file": "user/data/perturb.csv",
        "rows": [0, 1]
      },
      "vehicles": {
        "vehicle": {
          "inputs": ["engine.temp_level", "aero.drag_bias"]
        }
      }
    }
  },
  "outputs": {
    "directory": "user/outputs/simflow",
    "case_directory": "case_{case_index}"
  }
}
```

The built-in `simflow.materializer.numeric_perturbation` materializer supports
`single`, `matrix`, and `random` case sources:

```json
{
  "mode": "single",
  "inputs": {
    "engine.temp_level": 2,
    "aero.drag_bias": -0.03
  }
}
```

```json
{
  "mode": "matrix",
  "file": "user/data/perturb.csv",
  "rows": [0, 1]
}
```

```json
{
  "mode": "random",
  "seed": 12345,
  "count": 3,
  "inputs": {
    "engine.temp_level": { "distribution": "uniform_int", "min": 0, "max": 2 },
    "aero.drag_bias": { "distribution": "uniform", "min": -0.05, "max": 0.05 }
  }
}
```

`case_source.single.inputs` values must be finite numbers. The matrix file must have a
header row, `case_id` as the first column, and input columns after that.
Selected row indices are zero-based non-negative integers. `random.count` must
be positive, `random.seed` must be a non-negative integer, and random inputs
support numeric constants plus `constant`, `uniform`, `normal`, and
`uniform_int` distribution objects.

`materializer.config.vehicles.<vehicle_id>.inputs` lists which case variables
are injected into that vehicle's `vehicles[].perturbation.config.inputs`.
For each case, the runner also rewrites `outputs.directory` in the effective
mission to the current case directory while preserving the rest of the base
mission outputs config. A vehicle referenced by the built-in numeric
materializer must already define a `perturbation` block in the base mission.

Project-specific materializers can be added under
`user/<active_project>/simflows/*.hpp` and registered with
`GNC_REGISTER_SIMFLOW_MATERIALIZER`. A custom materializer receives the SimFlow
config, base mission, repo/project roots, output directory, and case-directory
pattern, then returns ordinary materialized cases.

Each generated case directory contains:

- `effective_mission.json`

The SimFlow output root contains:

- `simflow_summary.csv`

`effective_mission.json` is the replay contract for a generated case.
`simflow_summary.csv` contains
`case_index,case_id,case_directory,status,exit_code,message`.
In multiprocess mode, the
parent process generates all case files first, launches child processes with
`--config <case>/effective_mission.json`, continues after individual case
failures, and writes collected exit codes to `simflow_summary.csv`.

To replay a single SimFlow case, run its effective mission directly:

```powershell
gnc_sim.exe --config user/outputs/simflow/case_000001/effective_mission.json
```

## Outputs

| 字段 | 默认 | 说明 |
| --- | --- | --- |
| `enabled` | `true` | `false` 时关闭自动记录 |
| `directory` | `user/outputs` | 支持 `{timestamp}` |
| `format` | `csv` | 当前仅支持 CSV |
| `session_name` | `simulation_data` | 输出文件名前缀 |
| `precision` | `12` | CSV 数值精度 |
| `flush_every_step` | `false` | 是否每步 flush |
| `record_initial_state` | `true` | 是否记录 t0 发布态 |
| `record` | `all` | 记录规则 |
| `exclude` | `[]` | 排除完整字段或 `*.suffix` |
| `debug_snapshots` | disabled | 调试快照，可为 bool 或 object |

CSV `time` 列表示发布态物理时间。

CSV 行在 `update(t_k)` 之后写出：form/dynamics/truth view 字段是周期开始发布态 `x_k` 及其真实派生量；input/process/guidance/output 字段是 record 时刻组件暴露的本周期离散输出。`t0` 行包含初始状态 `x_0` 和基于 `x_0` 计算出的第一周期离散输出。框架不保证所有 observable 在物理意义上无延迟；延迟语义由组件模型自身定义。`outputs` 顶层未知字段只产生 build warning，不作为 build error。

`record` 可为 `"all"`、组件名数组，或 `{ "component.name": "all" | ["field_prefix"] }`。字段前缀会匹配嵌套 observable，例如 `velocity_launch` 匹配 `velocity_launch.x/y/z`。

`debug_snapshots` object 支持：

| 字段 | 默认 | 说明 |
| --- | --- | --- |
| `enabled` | `true` | 是否启用 |
| `components` | `all` | `"all"`、单个组件名或组件名数组 |
| `session_name` | `<session_name>_debug_snapshots` | debug CSV 文件名前缀 |
| `precision` | 继承 outputs | 数值精度 |
| `flush_every_step` | 继承 outputs | 是否每步 flush |

## Termination

`termination` 是顶层单例组件。旧 `stop_conditions[]` 不再支持。

支持：

- `termination.component_field_below`
- `termination.component_field_above`

字段：

| 字段 | 规则 |
| --- | --- |
| `component` | 必填完整组件名 |
| `field` | 必填 observable/state 字段 |
| `value` | 必填 number |
| `description` | 可选 string |

`summary` 是顶层单例 summary observer，组件必须实现 `ISummaryObserver`，其输出会追加到 `summary.txt`。
