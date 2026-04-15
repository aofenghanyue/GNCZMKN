# Documentation Index

## Quick Start

- [How To Add A New Component](guide-new-component.md)
- [Glossary](glossary.md)

## Architecture Decisions

- [ADR-001: Four-Layer Plugin Model](adr/ADR-001-four-layer-plugin-model.md)
- [ADR-002: Compile-Time Modularity](adr/ADR-002-compile-time-modularity.md)
- [ADR-003: Entity-First Mission Model](adr/ADR-003-entity-first-mission-model.md)
- [ADR-004: Fixed-Timestep Simulation Loop](adr/ADR-004-fixed-timestep-design.md)

## Diagrams

- [Component Assembly](diagrams/component-assembly.md)
- [Mission Topologies](diagrams/mission-topologies.md)
- [Runtime Sequence](diagrams/runtime-sequence.md)

## Current Architecture Baseline

- The simulation kernel stays in `common/`, `core/`, `infrastructure/`, and
  top-level `interfaces/`.
- Domain capabilities live under `framework/include/gnc/plugins/`.
- The current builtin plugin set is `environment`, `aero`, `state_3dof`, and
  `soviet_coord`.
- User-owned algorithm components live under `user/<project>/components/`.
- Mission assembly is entity-first:
  - environment components are registered as `env.*`,
  - vehicle components are registered as `<entity_id>.*`,
  - services are configured through `global_services.<plugin_name>` or
    `entities[i].services.<plugin_name>`.

## Code Entry Points

- Builtin plugin aggregator:
  [framework/include/gnc/plugins/_builtin_plugins.hpp](../framework/include/gnc/plugins/_builtin_plugins.hpp)
- Plugin registry:
  [framework/include/gnc/core/plugin_registry.hpp](../framework/include/gnc/core/plugin_registry.hpp)
- Simulation builder:
  [framework/include/gnc/core/simulation_builder.hpp](../framework/include/gnc/core/simulation_builder.hpp)
- Simulator:
  [framework/include/gnc/core/simulator.hpp](../framework/include/gnc/core/simulator.hpp)
