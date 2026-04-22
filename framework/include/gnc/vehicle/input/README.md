# Vehicle Input

`vehicle.input` is the measurement-side runtime layer.

It is reserved for:

- IMU and inertial measurement packages
- GPS, radar, seekers, pressure sensors, and other observation hardware
- synthetic measurement adapters built on top of form truth

This directory exists even when the active repository has no builtin input
packages yet. New input-side work should land here rather than being folded
into `vehicle.process` or `interaction`.
