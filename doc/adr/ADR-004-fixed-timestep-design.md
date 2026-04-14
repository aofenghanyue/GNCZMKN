# ADR-004: Fixed-Timestep Simulation Loop

## Status

Accepted

## Context

The current simulation loop assumes a fixed timestep defined by
`simulation.dt`.

This assumption is embedded in multiple places:

- `Simulator::run()` computes a fixed total step count from `duration / dt`.
- before-step and after-step callbacks receive `config_.dt`.
- simulation time advances by `config_.dt` every iteration.
- `component->update()` receives `config_.dt`.
- component execution-frequency scheduling is derived from a fixed simulation
  frequency.
- `AutoDataLogger` records at uniform step boundaries.

Returning `actual_dt` from `IIntegrator::step()` alone would not make the
simulation variable-step capable.

## Decision

The framework remains fixed-timestep for now.

`IIntegrator::step()` keeps its current `void` signature, and runtime code is
not changed as part of this phase.

## Consequences

- Adaptive-step integrators such as RK45 are not supported yet.
- Event timing remains step-aligned rather than integrator-adaptive.
- A future variable-step effort must redesign the simulation loop, not just the
  integrator interface.

## Revisit Conditions

Revisit this decision when one of the following becomes important:

- adaptive-step integration,
- sub-step event detection,
- stiff-system solvers that require timestep control.

At that point the refactor scope should include `Simulator::run()`, callback
semantics, component scheduling, logging, and summary reporting.
