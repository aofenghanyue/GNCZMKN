# Current Architecture

## Summary

The active framework model is:

- `Form`
- `Environment`
- `Vehicle`
- `Interaction`

This is an explicitly assembled architecture. It is not a plugin graph.

## Vehicle Boundary

`Vehicle` is split into four user-facing blocks:

- `common`
- `input`
- `process`
- `output`

The intended meaning is:

- `vehicle.common`: static assets, profile selection, parameter packs, file
  references, passive initialization data
- `vehicle.input`: measurement-side runtime hardware and sensor packages
- `vehicle.process`: navigation, guidance, control, timing logic, and command
  generation
- `vehicle.output`: runtime physical-effect subsystems such as propulsion,
  aerodynamic models, mass evolution, configuration switching, separation, and
  other effectors

Only `input`, `process`, and `output` are formal runtime stages.

`common` is not a hidden non-scheduled physics layer. It is the asset/profile
layer.

## Asset, Loader, Runtime Split

The framework now distinguishes three different responsibilities:

1. Static assets
   Example: JSON datasets under `framework/data/vehicles/`.
2. Loaders and parsers
   These are utility code, not components.
3. Runtime components
   These expose simulation-time capabilities such as `IAeroModel`,
   `IConstantMass`, or `IContinuousMass`.

Runtime aerodynamic and mass components belong to `vehicle.output`. They may
load asset files during `configure()` or `initialize()`, but the asset itself
is not the component.

## Interaction Boundary

`Interaction` is the form-aware closure layer.

It consumes:

- form truth
- environment queries
- process commands
- current runtime results from `vehicle.output`

It produces:

- the selected form input

`Interaction` does not own aerodynamic tables, mass definitions, or other
vehicle physical models. It consumes typed runtime capabilities from
`vehicle.output`.

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

Within `vehicle`, the active structure is:

```text
vehicle:
  common:
  input:
  process:
  output:
```

## Runtime Scheduling

Per-step execution order is:

1. `environment`
2. `vehicle.input`
3. `vehicle.process`
4. `vehicle.output`
5. `interaction`
6. `form`
7. outputs and stop conditions

This schedule is explicit in the runtime shell.

## Registration Model

Builtins are registered through explicit bootstrap functions in
`framework/include/gnc/bootstrap/register_builtin_packages.hpp`.

There is no active static-registration fallback path.

The bootstrap now includes formal hooks for:

- `registerVehicleCommonPackages(...)`
- `registerVehicleInputPackages(...)`
- `registerVehicleProcessPackages(...)`
- `registerVehicleOutputPackages(...)`

The `vehicle.input` hook exists even when there are no builtin input packages
yet. That keeps the architecture skeleton complete.

## Current Direction

The main architectural cleanup still in progress is:

- keeping `vehicle.common` strictly as the asset/profile layer
- growing `vehicle.input` as a first-class extension area
- moving runtime physical behavior into `vehicle.output`
- keeping `interaction` as a thin form-aware closure layer

That is the active direction for new framework work.
