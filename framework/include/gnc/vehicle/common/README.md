# Vehicle Common

`vehicle.common` is the asset and profile layer.

It is reserved for:

- reference vehicle profiles
- static parameter packs
- dataset references
- file-backed asset descriptors
- non-scheduled passive asset providers when they are truly needed

It is not the home for runtime physical behavior such as:

- aerodynamic closure
- mass evolution
- configuration switching
- stage separation

Those runtime responsibilities belong to `vehicle.output`.
