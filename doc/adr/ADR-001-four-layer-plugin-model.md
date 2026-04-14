# ADR-001: Four-Layer Plugin Model

## Status

Accepted

## Context

The framework needs a clear boundary between generic simulation-engine
mechanisms and GNC-domain capabilities. Without a shared model, it is easy to
blur the difference between:

- engine infrastructure,
- subsystem models,
- higher-level domain facilities,
- user-owned guidance, navigation, and control logic.

The repository already reflects this split in code, but the design was not
captured in an architectural decision record.

## Decision

The framework adopts a four-layer plugin model:

1. Layer 0: simulation kernel
2. Layer 1: hardware / subsystem plugins
3. Layer 2: system plugins
4. Layer 3: user GNC algorithms

```text
+--------------------------------------------------------------------------+
| Layer 3: user GNC algorithms                                              |
| - user-written guidance, navigation, and control components               |
| - consume lower-layer interfaces and services                             |
| - issue commands and mission-specific logic                               |
+--------------------------------------------------------------------------+
| Layer 2: system plugins                                                   |
| - build higher-level domain facilities from lower-layer interfaces         |
| - examples: coordinate-convention services such as soviet_coord           |
| - do not solve motion equations or model hardware directly                |
+--------------------------------------------------------------------------+
| Layer 1: hardware / subsystem plugins                                     |
| - model physical subsystems or engineering entities                       |
| - examples: environment, aero, state_3dof                                |
| - expose domain interfaces for the quantities they provide                |
+--------------------------------------------------------------------------+
| Layer 0: simulation kernel                                                |
| - lifecycle, registries, simulator loop, config, logging, math utilities  |
| - contains engine mechanisms only                                         |
| - does not contain domain-specific coordinate trees or physical models     |
+--------------------------------------------------------------------------+
```

## Layer Definitions

### Layer 0: Simulation Kernel

Contains:

- `common/`, `core/`, `infrastructure/`, and top-level `interfaces/`
- `ComponentBase`, `ComponentFactory`, `ComponentRegistry`
- `Simulator`, `ConfigManager`, integrator interfaces, logging infrastructure

Does not contain:

- concrete coordinate-tree implementations,
- domain-specific physical models,
- plugin-specific interfaces or services.

Decision rule:

- if removing it makes the engine unable to assemble or execute a simulation,
  it belongs to Layer 0.

### Layer 1: Hardware / Subsystem Plugins

Characteristics:

- each plugin models an engineering entity or subsystem,
- each plugin exposes the quantities that entity can provide,
- these plugins may solve motion equations or emulate subsystem behavior.

Current examples:

- `environment`
- `aero`
- `state_3dof`

Decision rule:

- if it solves equations of motion or models hardware/subsystem behavior, it
  belongs to Layer 1.

### Layer 2: System Plugins

Characteristics:

- build higher-level domain facilities from Layer-1 interfaces,
- do not directly model hardware,
- do not solve motion equations,
- are optional from the simulation-engine perspective.

Current example:

- `soviet_coord`

Decision rule:

- if it composes lower-layer interfaces into a domain facility, it belongs to
  Layer 2.

### Layer 3: User GNC Algorithms

Characteristics:

- project-owned algorithm components,
- mission-specific logic,
- consume lower-layer interfaces and services,
- live under `user/<project>/components/`.

Decision rule:

- if the user owns it as mission or algorithm logic, it belongs to Layer 3.

## Dependency Rules

- higher layers may depend on lower-layer interfaces,
- lower layers must not depend on higher layers,
- plugins at the same layer may depend on each other only when the dependency
  is explicit,
- Layer 0 must not include headers from `framework/include/gnc/plugins/`.

## Consequences

Positive:

- new capabilities have a consistent classification rule,
- engine code is less likely to absorb domain logic accidentally,
- plugin responsibilities are easier to explain to contributors.

Tradeoffs:

- architectural boundaries must be kept explicit in documentation and code
  review,
- some functionality that feels convenient in the short term must stay out of
  Layer 0 to keep the kernel stable.
