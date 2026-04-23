# Reference

This file is a quick reference for the current framework. For the mission
layout overview, start with [03-mission-configuration.md](03-mission-configuration.md).

## Command Line

| Command | Purpose |
| --- | --- |
| `gnc_sim.exe` | Run the active project's default mission |
| `gnc_sim.exe <config.json>` | Run the given mission file |
| `gnc_sim.exe --config <config.json>` | Run the given mission file |
| `gnc_sim.exe --list-components` | List registered component type ids |
| `gnc_sim.exe --list-components-verbose` | List type ids, interfaces, and registration origins |
| `gnc_sim.exe --help` | Print command help |

Mission paths may be absolute, relative to the current directory, or relative
to the repository root. The runner searches upward from the working directory
and executable directory.

## Mission Schema

Active missions use these top-level blocks:

| Path | Type | Purpose |
| --- | --- | --- |
| `simulation` | object | Runtime step, duration, integrator |
| `form` | object | Vehicle state/form components |
| `environment` | object | Environment services and components |
| `vehicle` | object | Vehicle services and `common/input/process/output` components |
| `interaction` | object | Form-aware closure from environment/process/output to form input |
| `outputs` | object | Automatic logging configuration |
| `stop_conditions` | array | Top-level stop condition list |
| `global_services` | object | Global service configuration |

`entities[]` and legacy root-level `components` / `services` / `vehicles` are
not supported by the active runtime.

## Common Mission Fields

| Path | Type | Purpose |
| --- | --- | --- |
| `simulation.dt` | number | Fixed step size in seconds |
| `simulation.duration` | number | Maximum simulation time in seconds |
| `simulation.integrator` | string | `rk4` or `euler` |
| `outputs.enabled` | bool | Enable or disable automatic logging |
| `outputs.directory` | string | Output directory; supports `{timestamp}` |
| `outputs.format` | string | Currently `csv` |
| `outputs.session_name` | string | Output filename prefix |
| `outputs.record` | string / array / object | Stable field recording rule |
| `outputs.exclude` | array | Full field names or `*.suffix` patterns to skip |

## Builtin Components

| Type id | Placement | Main interfaces |
| --- | --- | --- |
| `environment.spherical_earth` | `environment.components` | `IEarth` |
| `environment.wgs84_earth` | `environment.components` | `IEarth` |
| `environment.standard_atmosphere` | `environment.components` | `IAtmosphere` |
| `environment.spherical_gravity` | `environment.components` | `IGravity` |
| `form.cartesian_3dof.point_mass` | `form.components` | `IContinuousSystem`, `ITruthView`, `IObservable` |
| `form.local_spherical_3dof.point_mass` | `form.components` | `IContinuousSystem`, `ITruthView`, `IObservable` |
| `form.local_spherical_3dof.flight_state_view` | `form.components` | `IFlightStateView`, `IObservable` |
| `interaction.cartesian_3dof.direct_accel` | `interaction.components` | cartesian `IInputProvider` |
| `interaction.local_spherical_3dof.direct_accel` | `interaction.components` | local-spherical `IInputProvider` |
| `interaction.local_spherical_3dof.aero_propulsive` | `interaction.components` | local-spherical `IInputProvider` |
| `vehicle.process.programmed_aoa` | `vehicle.process` | `IAeroGuidanceProvider`, `IObservable` |
| `aero.simple_polynomial` | `vehicle.output` | `IAeroModel`, `IObservable` |
| `aero.table2d` | `vehicle.output` | `IAeroModel`, `IObservable` |
| `mass.constant` | `vehicle.output` | `IConstantMass`, `IObservable` |
| `mass.continuous_constant_rate` | `vehicle.output` | `IContinuousMass`, `IContinuousSystem`, `IObservable` |

Project example components are not builtin API. For example,
`example.coordinate_probe` lives under `user/example_03_coordinate_tree/components/`.

## Builtin Services

| Service id | Placement | Purpose |
| --- | --- | --- |
| `coordinate_tree` | `vehicle.services.coordinate_tree` | Installs `ICoordService` for coordinate-frame transforms |

`coordinate_tree` v1 is vehicle-scoped only. `global_services.coordinate_tree`
and `environment.services.coordinate_tree` are rejected intentionally.

Supported coordinate-tree specs:

| Spec id | Purpose |
| --- | --- |
| `empty` | Root frame `I`, no extra frames or edges |
| `local_spherical_3dof.launch_track` | Builds the current local-spherical launch/track frame tree |

`local_spherical_3dof.launch_track` requires:

| Path | Purpose |
| --- | --- |
| `spec` | Must be `local_spherical_3dof.launch_track` |
| `bindings.earth.name` | Component implementing `IEarth` |
| `bindings.truth.name` | Component implementing local-spherical `ITruthView` |
| `launch.latitude_rad` | Launch latitude |
| `launch.longitude_rad` | Launch longitude |
| `launch.azimuth_rad` | Launch azimuth |
| `launch.launch_time_s` | Launch time |
| `launch.earth_rotation_angle_rad` | Initial Earth rotation angle |

Coordinate-tree specs are owned by the coordinate-tree service package. Core
builders do not register or include concrete coordinate-tree specs.

## Naming Rules

| Object | Rule | Example |
| --- | --- | --- |
| Environment component full name | `env.<component_name>` | `env.atmosphere` |
| Vehicle component full name | `vehicle.<component_name>` | `vehicle.dynamics` |
| Same-vehicle dependency lookup | Local name or full name | `dynamics` |
| Cross-scope dependency lookup | Full name | `env.gravity` |
| Logs and stop conditions | Full component and field names | `vehicle.dynamics.altitude_m` |
