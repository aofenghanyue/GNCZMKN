# Glossary

| Term | Definition |
| --- | --- |
| **Component** | The basic simulation unit. Components inherit from `ComponentBase`, participate in the lifecycle (`configure`, service injection, dependency injection, `initialize`, `update`, `finalize`), and may implement one or more interfaces. |
| **Interface** | A pure capability contract, usually named with an `I*` prefix, that other components or services bind against, such as `IAtmosphere` or `IStateSolver3DOF`. |
| **Plugin** | A compile-time module that groups related interfaces, components, and optional service installers behind a single `Plugin::install()` entry point. |
| **Service** | A non-component facility stored in `ServiceContext` and installed by a plugin, such as `ISovietCoordService`. |
| **Entity** | A top-level mission assembly unit with an `id`, a `role`, optional `services`, and a `components` array. The current roles are `environment` and `vehicle`. |
| **Scope Prefix** | The entity-qualified component-name prefix used for global uniqueness and dependency lookup, such as `env.` or `missile.`. |
| **Registry** | The `ComponentRegistry` that stores runtime component instances and their exposed interfaces. `ScopedRegistry` provides scope-aware lookup on top of it. |
| **Simulation Kernel** | Layer 0 of the architecture: lifecycle, registries, simulator loop, logging, configuration, math types, and integrator interfaces. |
| **Subsystem Plugin** | A Layer-1 plugin that models a physical subsystem or engineering entity, such as `environment`, `aero`, or `state_3dof`. |
| **System Plugin** | A Layer-2 plugin that composes lower-layer interfaces into a higher-level facility rather than directly modeling hardware, such as `soviet_coord`. |
| **Project Component** | A user-owned component compiled from `user/<project>/components/`. These are Layer-3 mission or algorithm modules. |
| **Continuous System** | A component that implements `IContinuousSystem` and exposes a state vector plus derivative function for the integrator. |
| **Observable** | A component that implements `IObservable` so `AutoDataLogger` can record named fields automatically. |
| **Mission** | A JSON configuration file that defines simulation settings, outputs, global services, and the `entities[]` graph to assemble. |
| **Single-Vehicle Mission** | A mission with exactly one `role=vehicle` entity. In Chinese discussion this should be called `单飞行器`, not `单车`. |
| **Multi-Vehicle Mission** | A mission with more than one `role=vehicle` entity. In Chinese discussion this should be called `多飞行器`, not `多车`. |
| **Preflight Dependency Binding** | The builder-time pass that calls `injectDependencies()` against a diagnostic `ScopedRegistry` after all components and deferred service actions are registered. |
