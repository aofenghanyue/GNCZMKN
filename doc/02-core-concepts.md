# Core Concepts

## Four Top-Level Concepts

The framework is organized around four concepts:

- `Form`
- `Environment`
- `Vehicle`
- `Interaction`

`Form` owns propagation state and form-local truth/input surfaces.
`Environment` owns world queries.
`Vehicle` owns assembled onboard subsystems.
`Interaction` closes environment and vehicle behavior into form input.

## Vehicle Roles

The vehicle model is:

- `common`: static assets and profiles
- `input`: sensors and measurement-side hardware
- `process`: software and command generation
- `output`: runtime physical-effect subsystems

This distinction is intentional:

- `common` describes what data is selected
- `output` describes what runtime capability is active

Examples of `vehicle.output` responsibilities:

- aerodynamic model activation
- mass and inertia evolution
- propulsion
- control-surface effectors
- stage separation and configuration switching

## Asset vs Runtime Component

Do not confuse these layers:

- asset file
- loader/parser utility
- runtime component

An aerodynamic table JSON file is an asset.
A JSON loader is a utility.
`aero.table2d` is the runtime component that exposes aerodynamic capability
during simulation.

The same applies to mass models and future configuration-management systems.

## Interaction

`Interaction` is not a bag of vehicle physics.

Its job is narrower:

- read form truth
- query environment
- read process commands
- read output-layer runtime capabilities
- compute the selected form input

That is why runtime aero and mass models belong to `vehicle.output`, not to
`interaction`.

## Runtime Schedule

The formal runtime schedule is:

1. environment
2. vehicle.input
3. vehicle.process
4. vehicle.output
5. interaction
6. form

`vehicle.common` is intentionally outside this schedule.

## Registration

Component registration is explicit.

`GNC_REGISTER_COMPONENT_TYPE` is only valid when the build-generated explicit
registration chain defines `GNC_COMPONENT_REGISTRATION_FN`. There is no hidden
static fallback.
