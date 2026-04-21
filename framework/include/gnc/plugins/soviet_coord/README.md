# soviet_coord service package

`soviet_coord` remains a service package, but it is no longer part of an active
plugin architecture.

Current service installation is owned directly by assembly code through the
runtime shell rather than through `PluginRegistry`.

This package provides:

- `ISovietCoordService`
- the Soviet coordinate-chain construction logic used by the current
  local-spherical 3DoF examples

Some of its math is still service-side and some form-specific math has already
been moved back into `local_spherical_3dof`. New work should keep form-native
math inside the owning form package.
