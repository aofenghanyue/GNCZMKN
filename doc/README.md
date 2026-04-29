# Documentation Map

These documents describe the current GNCZMKN runtime and user contracts. When
code and docs differ, trust code and tests.

## First Run

- [01-getting-started.md](01-getting-started.md): build, test, run, inspect outputs.
- [user/README.md](../user/README.md): active project layout, project components, `user/data`, outputs.

## Mission Authoring

- [03-mission-configuration.md](03-mission-configuration.md): mission schema, `$include`, vehicles, outputs, termination, summary, coordinate-tree.
- [06-reference.md](06-reference.md): CLI, builtin type ids, service ids, common fields.

Useful examples:

- `user/example_01_minimal_pluginized/config/mission.json`
- `user/example_02_atmospheric_3dof/config/mission.json`
- `user/example_03_coordinate_tree/config/mission.json`

## Extensions

- [04-extension-guide.md](04-extension-guide.md): active project components, private interfaces, registration metadata, custom interaction examples.

## Maintainers

- [00-current-architecture.md](00-current-architecture.md): short architecture snapshot.
- [02-core-concepts.md](02-core-concepts.md): forms, vehicles, interactions, scheduling, naming.
- [05-architecture.md](05-architecture.md): assembly, services, validation, logging, scheduling data flow.
- [07-decisions.md](07-decisions.md): current architecture decisions.
