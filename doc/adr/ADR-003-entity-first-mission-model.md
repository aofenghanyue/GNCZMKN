# ADR-003: Entity-First Mission Model

## Status

Accepted

## Context

The framework previously split mission assembly across separate top-level
configuration paths such as:

- `environment`,
- `components`,
- `services`,
- `vehicles`.

That created different code paths for single-vehicle and multi-vehicle missions
and made service scope behavior inconsistent.

## Decision

Mission configuration is now entity-first.

Every mission uses an `entities[]` array. Each entity has:

- `id`,
- `role`,
- optional `services`,
- `components`.

The currently supported roles are:

- `environment`,
- `vehicle`.

Assembly rules:

- all environment entities are built before all vehicle entities,
- environment components are registered as `env.*`,
- vehicle components are registered as `<entity_id>.*`,
- vehicle services live in the vehicle's own `ServiceContext`,
- global facilities still use `global_services`.

## Naming And Scope Constraints

### Component Naming

- `env.*` is reserved for the environment entity namespace.
- vehicle-local component names are always expanded to `<entity_id>.<component_name>`.
- in a single-vehicle mission, the recommended vehicle entity id is `vehicle`.
- in a multi-vehicle mission, use semantic ids such as `missile`, `target`,
  `interceptor`, or `decoy`.
- stop conditions, output recording rules, and service bindings should use the
  fully scoped component names, not local shorthand.

Examples:

- single-vehicle: `vehicle.dynamics`
- multi-vehicle: `missile.dynamics`, `target.dynamics`
- environment: `env.atmosphere`

### Dependency Lookup

- relative lookup names such as `guidance` or `aero` resolve only within the
  current entity scope,
- cross-entity dependencies must use explicit scoped names such as
  `env.gravity`,
- cross-vehicle direct dependencies should be explicit and uncommon; if many
  vehicles need the same facility, that facility likely belongs in
  `global_services`.

### Service Visibility

- service injection order is `global -> environment -> vehicle`,
- global services are visible to every entity,
- environment services are visible to environment components and vehicle
  components,
- vehicle-local services are visible only inside that vehicle entity,
- a service should be promoted to `global_services` only when cross-vehicle
  sharing is intentional.

## Rationale

- Single-vehicle missions become a special case of the same entity model rather
  than a separate assembly path.
- Multi-vehicle simulation is treated as a first-class design target.
- Cross-entity dependencies become explicit through scoped names such as
  `env.atmosphere` and `missile.dynamics`.
- Service injection order becomes consistent: `global -> environment -> vehicle`.
- Naming rules become predictable enough for logging, stop conditions, and
  service bindings to share one convention.

## Consequences

Positive:

- one assembly model covers environment and vehicles,
- component names are globally unambiguous,
- service scope and dependency lookup behavior are easier to reason about.
- single-vehicle and multi-vehicle missions now follow the same naming rules.

Tradeoffs:

- legacy root-level mission formats are no longer the primary contract,
- single-vehicle missions now require an explicit vehicle entity and prefixed
  component names,
- only one environment entity is currently supported by the builder,
- users must write fully scoped names in mission outputs, stop conditions, and
  some service bindings.
