# Component Assembly

This page shows the build-time assembly path for the current entity-first
mission model.

## Build Flow

```mermaid
flowchart TD
    M["mission.json"] --> C["ConfigManager::loadFromFile()"]
    C --> B["SimulationBuilder::build()"]

    B --> S["Parse simulation settings and choose integrator"]
    B --> G["Install global services"]
    B --> E1["Pass 1: build all environment entities"]
    B --> E2["Pass 2: build all vehicle entities"]
    B --> D["Run deferred service actions"]
    B --> V["DependencyValidator + preflight injectDependencies()"]
    B --> T["Build stop conditions"]
    B --> L["Initialize AutoDataLogger"]
    B --> R{"Build diagnostics clean?"}
    R -- "yes" --> READY["Simulator ready"]
    R -- "no" --> FAIL["Throw build exception"]

    E1 --> ENV["buildEntity(environment)"]
    E2 --> VEH["buildEntity(vehicle)"]

    ENV --> ENV_SVC["buildServices(entity.services, scope='env')"]
    ENV --> ENV_COMP["registerComponents(prefix='env.')"]
    VEH --> VEH_SVC["buildServices(entity.services, scope='<id>')"]
    VEH --> VEH_COMP["registerComponents(prefix='<id>.')"]

    ENV_COMP --> COMP["ComponentFactory::create(type)"]
    VEH_COMP --> COMP
    COMP --> CFG["configure(config)"]
    CFG --> INJ["injectServices(global -> environment -> vehicle)"]
    INJ --> REG["ComponentRegistry::addDynamic(full_name, interfaces)"]
```

## Runtime Dependency Shape

```mermaid
graph LR
    subgraph ENV["Environment Entity"]
        EARTH["env.earth : IEarth"]
        ATMO["env.atmosphere : IAtmosphere"]
        GRAV["env.gravity : IGravity"]
    end

    subgraph MISSILE["Vehicle Entity: missile"]
        GUID["missile.guidance : IFlightCommandProvider3DOF"]
        AERO["missile.aero : IAeroModel"]
        DYN["missile.dynamics : IStateSolver3DOF + IVelocityDirectionProvider"]
    end

    subgraph SVC["Vehicle Service Context"]
        COORD["soviet_coord : ISovietCoordService"]
    end

    GUID --> DYN
    DYN --> AERO
    DYN --> ATMO
    DYN --> GRAV
    DYN -. optional .-> EARTH
    AERO -. optional .-> DYN
    AERO -. optional .-> GUID
    COORD --> EARTH
    COORD --> DYN
```

## Notes

- Environment entities are always assembled before vehicle entities, regardless
  of `entities[]` declaration order.
- Cross-entity component dependencies use explicit scoped names such as
  `env.atmosphere`.
- Same-entity dependencies may use local names such as `guidance`; they resolve
  through the requester's entity scope.
- Stop conditions, output recording rules, and service bindings should use full
  scoped names such as `vehicle.dynamics` or `missile.dynamics`.
- Component registration order inside one entity becomes runtime execution
  order inside `Simulator::step()`.
- `injectDependencies()` is not executed during `buildEntity()`. The builder
  first registers the whole mission graph, then validates and preflights
  bindings after deferred service actions complete.
