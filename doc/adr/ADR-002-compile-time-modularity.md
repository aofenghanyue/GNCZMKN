# ADR-002: Compile-Time Modularity

## Status

Accepted

## Context

The framework needs a modular structure for subsystem plugins, system plugins,
and project-owned algorithm components. Two broad options existed:

- runtime dynamic loading,
- compile-time composition with static registration.

The actual repository already behaves as a compile-time system:

- builtin plugins are linked into the executable,
- builtin component types are registered through `Plugin::install()`,
- project-owned components are compiled from `user/active_project`,
- mission JSON selects among already-compiled types rather than loading new
  binaries.

## Decision

The framework adopts compile-time modularity.

Builtin plugins are compiled into the executable and expose their types through
static plugin registration plus `Plugin::install()`. Project-owned components
remain compile-time additions selected by the active project.

## Rationale

- The simulation loop is performance-sensitive and benefits from a simple,
  statically linked deployment model.
- The current users are framework developers and researchers, not plugin
  marketplace consumers.
- Windows and MinGW support stay simpler without a runtime DLL loading story.
- Header-oriented plugin code keeps implementation and interface boundaries easy
  to inspect in one codebase.

## Consequences

Positive:

- one executable is enough to run the framework,
- plugin installation is deterministic and testable,
- build-time type inventory is easy to inspect with `--list-components`.

Tradeoffs:

- changing builtin plugin sets or switching `user/active_project` requires a
  rebuild,
- runtime hot-plugging and unloading are out of scope,
- the architecture should be described as compile-time modular rather than a
  runtime plugin platform.
