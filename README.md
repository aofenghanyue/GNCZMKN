# GNCZMKN

GNCZMKN is a simulation framework for guidance, navigation, and control work.
The active architecture is centered on:

- `Form`
- `Environment`
- `Vehicle`
- `Interaction`

The old plugin-centered architecture is no longer the active design model.
Plugin-era compatibility shims and legacy type ids have been removed from the
active code path.

## Current Runtime Model

The runtime shell is built around:

- `ComponentBase`
- `ComponentFactory`
- `IContinuousSystem`
- `IIntegrator`
- `ScopedRegistry`
- `ServiceContext`
- `MissionAssembler`
- `AutoDataLogger`
- `ConfigNode`

Per-step execution order is:

1. `environment`
2. `vehicle.input`
3. `vehicle.process`
4. `vehicle.output`
5. `interaction`
6. `form`
7. outputs and stop conditions

## Mission Shape

Active missions use this top-level structure:

```text
simulation:
form:
environment:
vehicle:
interaction:
outputs:
stop_conditions:
```

The legacy top-level `entities[]` shape is intentionally unsupported.

## Canonical Builtin Families

Current canonical builtin form and interaction types include:

- `form.cartesian_3dof.point_mass`
- `interaction.cartesian_3dof.direct_accel`
- `form.local_spherical_3dof.point_mass`
- `form.local_spherical_3dof.flight_state_view`
- `interaction.local_spherical_3dof.direct_accel`
- `interaction.local_spherical_3dof.aero_propulsive`

## Quick Start

Build on Windows with MinGW:

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

Run the default mission:

```powershell
build-mingw\bin\gnc_sim.exe
```

Run a specific mission:

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

List registered components:

```powershell
build-mingw\bin\gnc_sim.exe --list-components
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

## Repository Layout

| Path | Purpose |
| --- | --- |
| `framework/include/gnc/core/` | runtime shell, assembly, validation, logging, stop conditions |
| `framework/include/gnc/forms/` | canonical form packages |
| `framework/include/gnc/environment/` | world-query packages |
| `framework/include/gnc/vehicle/` | vehicle common/input/process/output packages |
| `framework/include/gnc/interactions/` | form-aware closure packages |
| `framework/include/gnc/plugins/` | compatibility headers and legacy namespaces only |
| `user/` | missions, active-project code, and outputs |
| `doc/` | current documentation entrypoint and archived reference docs |

## Documentation

Start here:

- [Current Architecture](doc/00-current-architecture.md)
- [Documentation Index](doc/README.md)
- [User Workspace Notes](user/README.md)

The files `doc/01-getting-started.md` through `doc/07-decisions.md` are
retained as archived reference from the older plugin-era documentation set.
They are not the source of truth for the current architecture.
