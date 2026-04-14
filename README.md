# GNC Simulation Framework

## Architecture

This repository is a compile-time modular C++ simulation framework for GNC
research. Components are integrated as header-only modules and builtin plugins
register their component types through `Plugin::install()`; there is no runtime
dynamic loading.

The framework follows a four-layer plugin model:

| Layer | Role | Examples |
| :---: | ---- | -------- |
| 0 | Simulation kernel | `ComponentBase`, `Simulator`, registries, config, logging |
| 1 | Hardware / subsystem plugins | `environment`, `aero`, `state_3dof` |
| 2 | System plugins | `soviet_coord` |
| 3 | User GNC algorithms | `user/<project>/components/` |

Core design principles:

- GNC researchers should focus on algorithm modules, not framework plumbing.
- The framework handles assembly, integration, logging, and dependency management.
- Environment entities are assembled before vehicle entities.
- Components execute in entity-local mission JSON declaration order.
- 3DOF and a future 6DOF stack are independent Layer-1 plugins rather than a
  shared state-interface hierarchy.

This repository now uses a plugin-oriented architecture built around four layers:

1. `common/`, `core/`, `infrastructure/`, and top-level `interfaces/` provide the simulation kernel.
2. `plugins/environment`, `plugins/aero`, `plugins/state_3dof`, and `plugins/soviet_coord` provide the first production plugin set.
3. `user/<project>/components` provides project-owned GNC algorithm components.
4. Mission JSON assembles entity-local components and services through stable
   plugin-qualified type identifiers.

## Current Baseline

- Continuous dynamics use `gnc::interfaces::IContinuousSystem`.
- Plugin service installation is handled by `gnc::core::PluginRegistry`.
- Builtin component types use qualified IDs such as:
  - `environment.wgs84_earth`
  - `aero.simple_polynomial`
  - `state_3dof.point_mass_spherical`
- Mission configuration is entity-first: environment components are registered
  as `env.*`, and vehicle components are registered as `<entity_id>.*`.
- Coordinate transformation services are configured through
  `global_services.<plugin_name>` or `entities[i].services.<plugin_name>`.

## Build

The repository is validated with CMake `MinGW Makefiles`.

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

## Run

The active project is controlled by [user/active_project](user/active_project).

- Default pluginized minimal mission:
  [user/config/missions/default.json](user/config/missions/default.json)
- Atmospheric 3DOF mission:
  [user/example_02_atmospheric_3dof/config/mission.json](user/example_02_atmospheric_3dof/config/mission.json)
- Soviet coordinate service probe:
  [user/example_03_soviet_coord/config/mission.json](user/example_03_soviet_coord/config/mission.json)

```powershell
build-mingw\\bin\\gnc_sim.exe
build-mingw\\bin\\gnc_sim.exe --list-components
build-mingw\\bin\\gnc_sim.exe --config user/config/missions/default.json
```

Project-owned components are still compiled from `user/active_project`.
To run a mission that depends on another project directory, update
`user/active_project`, rebuild, and then launch that mission.

## Reference

- [doc/README.md](doc/README.md)
- [doc/adr/ADR-001-four-layer-plugin-model.md](doc/adr/ADR-001-four-layer-plugin-model.md)
- [doc/adr/ADR-004-fixed-timestep-design.md](doc/adr/ADR-004-fixed-timestep-design.md)
- [doc/guide-new-component.md](doc/guide-new-component.md)
- [framework/include/gnc/plugins/_builtin_plugins.hpp](framework/include/gnc/plugins/_builtin_plugins.hpp)
- [framework/include/gnc/core/plugin_registry.hpp](framework/include/gnc/core/plugin_registry.hpp)
