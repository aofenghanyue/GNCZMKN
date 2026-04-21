# Current Architecture

## Summary

The active architecture is:

- `Form`
- `Environment`
- `Vehicle`
- `Interaction`

This replaced the earlier plugin-centered design as the primary framework
model.

## Top-Level Concepts

### Form

`Form` owns:

- state definition
- propagation equations
- form-local typed truth surfaces
- form-local typed input surfaces
- form-native derived quantities
- form-local math and validation

Current canonical form families include:

- `cartesian_3dof`
- `local_spherical_3dof`

### Environment

`Environment` provides world queries such as:

- Earth models
- atmosphere models
- gravity models

Environment packages should stay world-side and should not absorb vehicle or
interaction closure logic.

### Vehicle

`Vehicle` follows the user-facing assembly model:

- `common`
- `input`
- `process`
- `output`

Only `input`, `process`, and `output` are formal runtime stages. `common`
contains passive shared models and data.

### Interaction

`Interaction` is the explicit closure layer that translates:

- selected form truth
- environment queries
- vehicle outputs and relevant vehicle state

into the selected form input.

Examples:

- `interaction.cartesian_3dof.direct_accel`
- `interaction.local_spherical_3dof.direct_accel`
- `interaction.local_spherical_3dof.aero_propulsive`

## Mission Schema

Active missions use:

```text
simulation:
form:
environment:
vehicle:
interaction:
outputs:
stop_conditions:
```

The legacy `entities[]` schema is intentionally rejected.

## Runtime Scheduling

Per-step execution order is:

1. `environment`
2. `vehicle.input`
3. `vehicle.process`
4. `vehicle.output`
5. `interaction`
6. `form`
7. outputs and stop conditions

This schedule is explicit in the runtime shell and is no longer derived from
component registration order.

## Registration Model

Builtins are registered through explicit bootstrap functions in
`framework/include/gnc/bootstrap/register_builtin_packages.hpp`.

There is no active `PluginRegistry` path.

`ComponentFactory` metadata now includes:

- package role
- execution stage
- form family

This metadata is used during assembly and validation.

## Canonical vs Compatibility Names

Canonical type ids are the preferred public names for new work.

Examples:

- `form.cartesian_3dof.point_mass`
- `form.local_spherical_3dof.point_mass`
- `form.local_spherical_3dof.flight_state_view`
- `interaction.cartesian_3dof.direct_accel`
- `interaction.local_spherical_3dof.direct_accel`
- `interaction.local_spherical_3dof.aero_propulsive`

Legacy plugin-era type ids are no longer registered. Active missions and code
should use only the canonical type ids shown above.

## Compatibility Policy

There is no active compatibility layer for the removed plugin architecture.

Do not author new code against:

1. old include paths under `framework/include/gnc/plugins/`
2. old plugin-era type ids

Historical plugin-era material may still appear in archived documents, but it
is not part of the supported repository architecture.

## Current Direction

The repository has already validated:

- explicit bootstrap instead of plugin registration
- staged runtime execution
- a full `local_spherical_3dof` vertical slice
- a second `cartesian_3dof` form under the same shell

Remaining cleanup work is mostly about removing or quarantining historical
plugin-era documentation wording from archived reference material.
