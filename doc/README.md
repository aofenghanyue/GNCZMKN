# Plugin Architecture Reference

The repository implementation follows the plugin architecture rethink baseline:

- The simulation kernel keeps only engine-level mechanisms in `common/`, `core/`, `infrastructure/`, and top-level `interfaces/`.
- Domain capabilities are implemented as plugins under `framework/include/gnc/plugins/`.
- The first supported plugin set is:
  - `environment`
  - `aero`
  - `state_3dof`
  - `soviet_coord`
- User-owned GNC algorithms remain in `user/<project>/components/`.

## Key Rules

- Mission configuration uses an `entities[]` array with stable
  plugin-qualified type names.
- Environment components are registered as `env.*`; vehicle components are
  registered as `<entity_id>.*`.
- Service configuration uses `global_services.<plugin_name>` or
  `entities[i].services.<plugin_name>`.
- The Soviet coordinate service owns the `I / E / N / L / LI / B / K / V` coordinate-system chain.
- The lower coordinate-system chain is activated by provider interfaces, not by hardcoding a specific degree-of-freedom model.

## Entry Points

- Builtin plugin aggregator:
  [framework/include/gnc/plugins/_builtin_plugins.hpp](../framework/include/gnc/plugins/_builtin_plugins.hpp)
- Plugin registry:
  [framework/include/gnc/core/plugin_registry.hpp](../framework/include/gnc/core/plugin_registry.hpp)
- Simulation builder:
  [framework/include/gnc/core/simulation_builder.hpp](../framework/include/gnc/core/simulation_builder.hpp)
