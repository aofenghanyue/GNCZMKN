# GNC Simulation Framework

This repository now uses a plugin-oriented architecture built around four layers:

1. `common/`, `core/`, `infrastructure/`, and top-level `interfaces/` provide the simulation kernel.
2. `plugins/environment`, `plugins/aero`, `plugins/state_3dof`, and `plugins/soviet_coord` provide the first production plugin set.
3. `user/<project>/components` provides project-owned GNC algorithm components.
4. Mission JSON assembles components and services through stable plugin-qualified type identifiers.

## Current Baseline

- Continuous dynamics use `gnc::interfaces::IContinuousSystem`.
- Plugin service installation is handled by `gnc::core::PluginRegistry`.
- Builtin component types use qualified IDs such as:
  - `environment.wgs84_earth`
  - `aero.simple_polynomial`
  - `state_3dof.point_mass_spherical`
- Coordinate transformation services are configured through `services.soviet_coord`.

## Build

The repository is validated with CMake `MinGW Makefiles`.

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

## Run

The active project is controlled by [user/active_project](/C:/Users/17721/.codex/worktrees/bc05/GNCZMKN/user/active_project).

- Default pluginized minimal mission:
  [user/config/missions/default.json](/C:/Users/17721/.codex/worktrees/bc05/GNCZMKN/user/config/missions/default.json)
- Atmospheric 3DOF mission:
  [user/example_02_atmospheric_3dof/config/mission.json](/C:/Users/17721/.codex/worktrees/bc05/GNCZMKN/user/example_02_atmospheric_3dof/config/mission.json)
- Soviet coordinate service probe:
  [user/example_03_soviet_coord/config/mission.json](/C:/Users/17721/.codex/worktrees/bc05/GNCZMKN/user/example_03_soviet_coord/config/mission.json)

```powershell
build-mingw\\bin\\gnc_sim.exe
build-mingw\\bin\\gnc_sim.exe --list-components
build-mingw\\bin\\gnc_sim.exe --config user/config/missions/default.json
```

Project-owned components are still compiled from `user/active_project`.
To run a mission that depends on another project directory, update
`user/active_project`, rebuild, and then launch that mission.

## Reference

- [doc/README.md](/C:/Users/17721/.codex/worktrees/bc05/GNCZMKN/doc/README.md)
- [framework/include/gnc/plugins/_builtin_plugins.hpp](/C:/Users/17721/.codex/worktrees/bc05/GNCZMKN/framework/include/gnc/plugins/_builtin_plugins.hpp)
- [framework/include/gnc/core/plugin_registry.hpp](/C:/Users/17721/.codex/worktrees/bc05/GNCZMKN/framework/include/gnc/core/plugin_registry.hpp)
