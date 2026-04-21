# environment compatibility layer

Canonical environment code now lives under:

- `gnc/environment/components/`
- `gnc/environment/interfaces/`

This `gnc/plugins/environment/` path is retained only as a compatibility layer
for older include paths and namespace aliases.

Current builtin environment types are still:

- `environment.spherical_earth`
- `environment.wgs84_earth`
- `environment.standard_atmosphere`
- `environment.spherical_gravity`

But the active architecture is no longer "pluginized mission assembly".
