# User Workspace

`user/` is the project workspace for mission assembly and project-owned GNC algorithms.

- `user/active_project` selects the active project compiled into `gnc_sim`.
- `user/<project>/components/` stores project-owned components.
- `user/<project>/config/mission.json` stores the project mission definition.
- `user/config/missions/default.json` provides a repository-level fallback mission.
- `user/outputs/` stores simulation output products.

## Current Examples

- `example_01_minimal_pluginized`
  A minimal `form.cartesian_3dof.point_mass + interaction.cartesian_3dof.direct_accel`
  mission.
- `example_02_atmospheric_3dof`
  The primary atmospheric local-spherical 3DoF chain:
  `environment + vehicle.process + vehicle.output + form.local_spherical_3dof +
  interaction.local_spherical_3dof.aero_propulsive`.
- `example_03_coordinate_tree`
  A service-focused local-spherical 3DoF mission that probes the coordinate-tree
  transformation chain.

Only the active project selected in `user/active_project` is auto-registered into
`gnc_sim`. If you switch from `example_02_atmospheric_3dof` to
`example_03_coordinate_tree`, rebuild the binary before running that mission.
