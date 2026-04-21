# aero compatibility layer

Canonical aerodynamic models now live under:

- `gnc/vehicle/common/components/`
- `gnc/vehicle/common/interfaces/`

This directory remains only as a compatibility layer for older include paths
and namespace aliases.

For new code, prefer the canonical vehicle-common path and treat `aero` as a
vehicle package role, not as a top-level plugin architecture category.
