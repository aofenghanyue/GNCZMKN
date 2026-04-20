# User Workspace

`user/` is the project workspace for mission assembly and project-owned GNC algorithms.

- `user/active_project` selects the active project compiled into `gnc_sim`.
- `user/<project>/components/` stores project-owned components.
- `user/<project>/config/mission.json` stores the project mission definition.
- `user/config/missions/default.json` provides a repository-level fallback mission.
- `user/outputs/` stores simulation output products.

## Current Examples

- `example_01_minimal_pluginized`
  A minimal `state_3dof.point_mass_cartesian` mission.
- `example_02_atmospheric_3dof`
  The primary atmospheric 3DOF builtin chain:
  `environment + aero + state_3dof + soviet_coord + project guidance`.
- `example_03_soviet_coord`
  A service-focused mission that probes the Soviet coordinate transformation chain.

Only the active project selected in `user/active_project` is auto-registered into
`gnc_sim`. If you switch from `example_02_atmospheric_3dof` to
`example_03_soviet_coord`, rebuild the binary before running that mission.
