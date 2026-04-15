# Mission Topologies

This page gives concrete topology examples for the two mission shapes discussed
most often in design reviews:

- single-vehicle missions,
- multi-vehicle missions.

For Chinese discussions, these correspond to:

- `single-vehicle` = `单飞行器`,
- `multi-vehicle` = `多飞行器`.

## Single-Vehicle Mission

The smallest entity-first mission usually contains:

- one environment entity,
- one vehicle entity,
- optional vehicle-local services such as `soviet_coord`.

```mermaid
graph TD
    subgraph ENV["Environment Entity"]
        E1["env.earth"]
        E2["env.atmosphere"]
        E3["env.gravity"]
    end

    subgraph VEH["Vehicle Entity: vehicle"]
        V1["vehicle.guidance"]
        V2["vehicle.aero"]
        V3["vehicle.dynamics"]
    end

    subgraph SVC["Vehicle Service Context"]
        S1["soviet_coord"]
    end

    V1 --> V3
    V2 -. optional .-> V1
    V2 -. optional .-> V3
    V3 --> E2
    V3 --> E3
    V3 -. optional .-> E1
    S1 --> E1
    S1 --> V3
```

Minimal JSON shape:

```json
{
  "global_services": {},
  "entities": [
    {
      "id": "vehicle",
      "role": "vehicle",
      "services": {
        "soviet_coord": {
          "bindings": {
            "earth": { "name": "env.earth" },
            "velocity_direction": { "name": "vehicle.dynamics" }
          }
        }
      },
      "components": [
        { "type": "example.guidance", "name": "guidance", "config": {} },
        { "type": "aero.simple_polynomial", "name": "aero", "config": {} },
        { "type": "state_3dof.point_mass_spherical", "name": "dynamics", "config": {} }
      ]
    },
    {
      "id": "environment",
      "role": "environment",
      "components": [
        { "type": "environment.wgs84_earth", "name": "earth", "config": {} },
        { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} },
        { "type": "environment.spherical_gravity", "name": "gravity", "config": {} }
      ]
    }
  ]
}
```

Key points:

- even in a single-vehicle mission, component names stay prefixed as
  `vehicle.*`,
- cross-entity dependencies are explicit,
- stop conditions and logging rules should reference the full names such as
  `vehicle.dynamics`.

## Multi-Vehicle Mission

A multi-vehicle mission extends the same model rather than switching to a
different assembly path.

```mermaid
graph TD
    subgraph ENV["Environment Entity"]
        E1["env.earth"]
        E2["env.atmosphere"]
        E3["env.gravity"]
    end

    subgraph MISSILE["Vehicle Entity: missile"]
        M1["missile.guidance"]
        M2["missile.aero"]
        M3["missile.dynamics"]
    end

    subgraph TARGET["Vehicle Entity: target"]
        T1["target.guidance"]
        T2["target.aero"]
        T3["target.dynamics"]
    end

    subgraph GLOBAL["Global Service Context"]
        G1["shared services"]
    end

    M1 --> M3
    M2 -. optional .-> M1
    M2 -. optional .-> M3
    T1 --> T3
    T2 -. optional .-> T1
    T2 -. optional .-> T3
    M3 --> E2
    M3 --> E3
    T3 --> E2
    T3 --> E3
    G1 -. injected .-> M1
    G1 -. injected .-> T1
```

Typical JSON shape:

```json
{
  "global_services": {
    "shared_service": {}
  },
  "entities": [
    {
      "id": "missile",
      "role": "vehicle",
      "components": [
        { "type": "example.guidance", "name": "guidance", "config": {} },
        { "type": "aero.simple_polynomial", "name": "aero", "config": {} },
        { "type": "state_3dof.point_mass_spherical", "name": "dynamics", "config": {} }
      ]
    },
    {
      "id": "target",
      "role": "vehicle",
      "components": [
        { "type": "example.guidance", "name": "guidance", "config": {} },
        { "type": "aero.simple_polynomial", "name": "aero", "config": {} },
        { "type": "state_3dof.point_mass_spherical", "name": "dynamics", "config": {} }
      ]
    },
    {
      "id": "environment",
      "role": "environment",
      "components": [
        { "type": "environment.wgs84_earth", "name": "earth", "config": {} },
        { "type": "environment.standard_atmosphere", "name": "atmosphere", "config": {} },
        { "type": "environment.spherical_gravity", "name": "gravity", "config": {} }
      ]
    }
  ]
}
```

Key points:

- every vehicle keeps its own local namespace,
- same-role components may reuse the same local names because the prefixes keep
  them distinct,
- the builder still assembles environment first, then all vehicles,
- global services are shared, while vehicle services stay vehicle-local unless
  deliberately moved to `global_services`.

## Naming Guidance

- Use `vehicle.*` only for an actual single-vehicle mission.
- Use semantic ids such as `missile`, `target`, `interceptor`, or `decoy` for
  multi-vehicle missions.
- Keep logging, stop conditions, and service bindings aligned with the full
  scoped component names.
