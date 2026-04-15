# Runtime Sequence

This page captures the current fixed-timestep runtime behavior after the
entity-first refactor.

## Full Lifecycle

```mermaid
sequenceDiagram
    participant USER as User
    participant BUILDER as SimulationBuilder
    participant SIM as Simulator
    participant REG as ComponentRegistry
    participant LOG as AutoDataLogger

    USER->>BUILDER: loadConfig("mission.json")
    USER->>BUILDER: build()
    BUILDER-->>USER: Simulator&

    USER->>SIM: run()

    opt "simulator not initialized yet"
        SIM->>REG: injectDependencies() for components not preflight-marked
        SIM->>REG: initialize() in registry order
        SIM->>LOG: initialize output session
    end

    loop "for step in [0, duration / dt)"
        SIM->>SIM: before_step callbacks
        SIM->>SIM: step(step_index)
        SIM->>LOG: recordStep(current_time)
        SIM->>SIM: after_step callbacks
        SIM->>SIM: current_time += dt
        SIM->>SIM: evaluate termination conditions
    end

    SIM->>REG: finalize() in registry order
    SIM-->>USER: completed / terminated
```

## One Simulation Step

```mermaid
sequenceDiagram
    participant SIM as Simulator
    participant COMP as ComponentBase
    participant CS as IContinuousSystem
    participant INT as IIntegrator

    loop "for each component in registry order"
        SIM->>COMP: setSimTimeInternal_(time, step)
        SIM->>COMP: shouldExecute(step)?

        alt "continuous component"
            SIM->>CS: getState()
            SIM->>INT: step(f, time, x, dt)
            loop "integrator stages"
                INT->>CS: computeDerivatives(t_stage, x_stage, dxdt)
            end
            INT-->>SIM: x updated in place
            SIM->>CS: setState(x_new)
            SIM->>COMP: update(dt)
        else "discrete component"
            SIM->>COMP: update(dt)
        else "skipped by frequency gate"
            SIM-->>COMP: no update this step
        end
    end
```

## Notes

- The simulation loop is fixed-timestep. `dt` stays equal to
  `simulation.dt` for callbacks, integration, logging, and stop-condition
  checks.
- Registry order matters for component updates. A guidance component should be
  declared before the dynamics component that consumes its command output.
- `AutoDataLogger` records after `Simulator::step()` and before
  `current_time += dt`.
