# Extension Guide

## Project Components

Project components live in:

```text
user/<project>/components/
```

Example:

```text
user/example_03_coordinate_tree/components/
```

These headers are collected by CMake into a build-generated explicit
registration chain.

## Registration Rule

Use `GNC_REGISTER_COMPONENT_TYPE(...)` only inside project component headers
that are discovered by the build.

Project registrations must declare package role, execution stage, and form
family metadata. Use `""` when the component is form-neutral.

There is no hidden static-registration fallback anymore.

If the build-generated registration chain does not define
`GNC_COMPONENT_REGISTRATION_FN`, the macro fails at compile time.

## Coordinate-tree Specs

`services.coordinate_tree.spec` is not a project auto-registration extension
point. Coordinate-tree specs are builtin service assets compiled into the
framework-side `coordinate_tree` service.

If you need to add a new coordinate-tree spec, implement it under:

```text
framework/include/gnc/services/coordinate_tree/specs/
```

and register it explicitly in:

```text
framework/include/gnc/services/coordinate_tree/specs/register_builtin_specs.hpp
```

## Choosing the Correct Vehicle Block

Place new components by responsibility:

- `vehicle.common`: static profile or asset-provider objects only
- `vehicle.input`: sensors and measurement-side hardware
- `vehicle.process`: software and command generation
- `vehicle.output`: runtime physical-effect subsystems

Do not place runtime aero or mass behavior in `vehicle.common`.

## Minimal Example

```cpp
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"

class StepCounter final : public gnc::core::ComponentBase,
                          public gnc::interfaces::IObservable {
public:
    StepCounter() : ComponentBase("StepCounter") {}

    void configure(const gnc::core::ConfigNode& config) override {
        increment_ = config["increment"].asDouble(increment_);
    }

    void update(double) override {
        count_ += increment_;
    }

    std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
        gnc::core::ObservableFieldBuilder builder;
        builder.addScalar("count", [this]() { return count_; });
        return builder.build();
    }

private:
    double increment_ = 1.0;
    double count_ = 0.0;
};

GNC_REGISTER_COMPONENT_TYPE("example.step_counter",
                            StepCounter,
                            ::gnc::core::ComponentPackageRole::VehicleProcess,
                            ::gnc::core::ExecutionStage::VehicleProcess,
                            "",
                            gnc::interfaces::IObservable)
```

## Mission Placement Example

```json
{
  "vehicle": {
    "common": [],
    "input": [],
    "process": [
      {
        "type": "example.step_counter",
        "name": "counter",
        "config": {
          "increment": 2.0
        }
      }
    ],
    "output": []
  }
}
```

## Builtin Extensions

Builtin framework extensions should land in the explicit package layout:

```text
framework/include/gnc/
  environment/
  forms/
  vehicle/common/
  vehicle/input/
  vehicle/process/
  vehicle/output/
  interactions/
```

If you add a builtin input-side package, wire it through
`registerVehicleInputPackages(ComponentFactory&)`.
