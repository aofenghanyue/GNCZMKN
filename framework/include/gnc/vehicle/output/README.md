# Vehicle Output

`vehicle.output` is the runtime physical-effect layer.

It owns subsystems that change the externally visible behavior of the vehicle,
including:

- propulsion
- aerodynamic models active at runtime
- control surfaces and other effectors
- mass evolution
- center-of-mass and inertia changes
- configuration switching and staging/separation

`interaction` consumes these runtime capabilities and closes them into a
form-specific input.
