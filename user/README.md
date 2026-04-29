# User Workspace

`user/` is the repository workspace for project missions, project components,
project-private headers, shared data assets, and simulation outputs.

## Layout

| Path | Purpose |
| --- | --- |
| `user/active_project` | Fallback active project name used by CMake when `-DGNC_ACTIVE_PROJECT=...` is not set |
| `user/<project>/config/mission.json` | Default mission for the active project |
| `user/<project>/components/` | Project component headers scanned for auto-registration |
| `user/<project>/interfaces/` | Project-private interfaces; included by `gnc_sim`, not scanned for components |
| `user/<project>/common/` | Project-private shared helpers; included by `gnc_sim`, not scanned for components |
| `user/<project>/include/` | Project-private include root; included by `gnc_sim`, not scanned for components |
| `user/data/` | Cross-project data assets referenced with `user-data://...` |
| `user/outputs/` | Runtime output directory |

`components/` must contain only registrable component headers with
`GNC_REGISTER_COMPONENT_TYPE`. Shared project contracts and utilities belong in
`interfaces/`, `common/`, or `include/`.

## Active Project

CMake chooses the active project in this order:

1. `-DGNC_ACTIVE_PROJECT=<project>` cache value.
2. `user/active_project`.

If both are present and different, CMake warns and uses the cache value. An
active project must provide `config/mission.json`; otherwise configuration
fails. If no active project is configured, `gnc_sim` has no default mission and
must be run with `--config <path>`.

Current repository default:

```text
example_02_atmospheric_3dof
```

Only the active project's component headers are auto-registered into `gnc_sim`.
After changing `user/active_project`, `-DGNC_ACTIVE_PROJECT`, or component
headers, rerun CMake configuration and rebuild.

## Examples

| Project | Purpose |
| --- | --- |
| `example_01_minimal_pluginized` | Minimal Cartesian 3DoF mission |
| `example_02_atmospheric_3dof` | Main atmospheric local-spherical 3DoF example |
| `example_03_coordinate_tree` | Coordinate-tree service plus project component example |

## Common Commands

Run the active project mission:

```powershell
build-mingw\bin\gnc_sim.exe
```

Run an explicit mission:

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

List registered components:

```powershell
build-mingw\bin\gnc_sim.exe --list-components-verbose
```
