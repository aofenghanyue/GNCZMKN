# state_3dof compatibility layer

This directory is no longer the primary architectural home for 3DoF forms.

Current canonical packages live under:

- `gnc/forms/cartesian_3dof/`
- `gnc/forms/local_spherical_3dof/`
- `gnc/interactions/local_spherical_3dof/`

The headers in `gnc/plugins/state_3dof/` remain only as compatibility aliases
for older include paths, interfaces, and legacy type ids.

Prefer canonical type ids for new work:

- `form.cartesian_3dof.point_mass`
- `form.local_spherical_3dof.point_mass`
- `form.local_spherical_3dof.flight_state_view`
- `interaction.cartesian_3dof.direct_accel`
- `interaction.local_spherical_3dof.direct_accel`
- `interaction.local_spherical_3dof.aero_propulsive`
