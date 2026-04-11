# soviet_coord plugin

Provides a coordinate transformation service that builds the Soviet coordinate-system chain.

- Service interface: `ISovietCoordService`
- Service config key: `services.soviet_coord`
- Upper chain:
  - `I / E / N / L / LI`
- Provider-driven lower chain:
  - `L / B`
  - `L / K`
  - `B / V`
