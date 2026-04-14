# How To Add A New Component

This guide shows how to add a project-owned component under
`user/<project>/components/` and wire it into a mission JSON file.

The examples use the same patterns as
`user/example_02_atmospheric_3dof/components/programmed_aoa_guidance.hpp`.

## Quick Start

1. Create a new header under `user/<project>/components/`.
2. Inherit from `gnc::core::ComponentBase`.
3. Implement `update(double dt)`.
4. Register the type with `GNC_REGISTER_COMPONENT_TYPE`.
5. Add the component to `mission.json`.
6. Rebuild and run the mission.

Minimal example:

```cpp
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"

class MyComponent final : public gnc::core::ComponentBase {
public:
    MyComponent() : ComponentBase("MyComponent") {}

    void update(double) override {
        // Mission logic runs here.
    }
};

GNC_REGISTER_COMPONENT_TYPE("example.my_component", MyComponent)
```

Mission entry:

```json
{
  "components": [
    {
      "type": "example.my_component",
      "name": "my_component",
      "config": {}
    }
  ]
}
```

## Lifecycle

The framework drives a component through this lifecycle:

1. `configure(const ConfigNode&)`
2. `injectServices(ServiceContext&)`
3. `injectDependencies(ScopedRegistry&)`
4. `initialize()`
5. `update(double dt)`
6. `finalize()`

Typical intent of each hook:

- `configure()`: read static configuration values from JSON.
- `injectServices()`: access services installed by plugins.
- `injectDependencies()`: bind required and optional component dependencies.
- `initialize()`: allocate runtime state or validate assumptions.
- `update()`: execute mission logic during the simulation loop.
- `finalize()`: flush or summarize component-owned resources.

`injectDependencies()` should stay side-effect free. The framework may preflight
bindings during build to produce better diagnostics before runtime.

Example:

```cpp
void configure(const gnc::core::ConfigNode& config) override {
    gain_ = config["gain"].asDouble(gain_);
}

void initialize() override {
    accumulated_error_ = 0.0;
}

void update(double dt) override {
    accumulated_error_ += dt;
}
```

## Dependency Injection

Use `ScopedRegistry` in `injectDependencies()` to bind component-to-component
dependencies by name:

```cpp
void injectDependencies(gnc::core::ScopedRegistry& registry) override {
    registry.bindAll(
        gnc::core::bind(required_dep_, "dependency_name"),
        gnc::core::bindIfPresent(optional_dep_, "optional_dependency"));
}
```

Rules:

- `bind(...)` is for required dependencies.
- `bindIfPresent(...)` is for optional dependencies.
- names are resolved relative to the component scope when no explicit prefix is
  given.

Important:

- components execute in mission JSON declaration order, so keep provider
  components earlier than the components that depend on them.

## Services

Use `injectServices()` for plugin-installed services:

```cpp
void injectServices(gnc::core::ServiceContext& services) override {
    coord_service_ = services.get<gnc::plugins::soviet_coord::ISovietCoordService>();
}
```

Use services when the dependency is an infrastructure-style facility provided by
the plugin system rather than another mission component.

## Observable Fields

Implement `gnc::interfaces::IObservable` to expose fields for
`AutoDataLogger`:

```cpp
std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
    gnc::core::ObservableFieldBuilder builder;
    builder.addScalar("error", [this]() { return error_; });
    return builder.build();
}
```

If the mission `outputs.record` configuration includes your component, the
framework will automatically record these fields.

## Configuration Parameters

Read config values in `configure()`:

```cpp
void configure(const gnc::core::ConfigNode& config) override {
    target_altitude_m_ = config["target_altitude_m"].asDouble(target_altitude_m_);
    enabled_ = config["enabled"].asBool(true);
}
```

The framework tracks accessed config keys and can warn about unused keys. This
helps catch spelling mistakes in mission JSON.

## Continuous Dynamics Components

If the component is a continuous-time dynamic system, implement
`gnc::interfaces::IContinuousSystem` in addition to `ComponentBase`.

Typical pattern:

```cpp
class MyDynamics final : public gnc::core::ComponentBase,
                         public gnc::interfaces::IContinuousSystem {
public:
    MyDynamics() : ComponentBase("MyDynamics") {
        x_index_ = layout_.addVariable("x");
        v_index_ = layout_.addVariable("v");
        state_ = Eigen::VectorXd::Zero(layout_.dimension());
    }

    const gnc::core::StateLayout& getStateLayout() const override {
        return layout_;
    }

    void computeDerivatives(double,
                            const Eigen::VectorXd& state,
                            Eigen::VectorXd& derivative) const override {
        derivative = Eigen::VectorXd::Zero(layout_.dimension());
        derivative[x_index_] = state[v_index_];
        derivative[v_index_] = -0.1 * state[v_index_];
    }

    const Eigen::VectorXd& getState() const override { return state_; }
    void setState(const Eigen::VectorXd& state) override { state_ = state; }
    Eigen::VectorXd getInitialState() const override { return state_; }

    void update(double) override {}

private:
    gnc::core::StateLayout layout_;
    Eigen::VectorXd state_;
    int x_index_ = -1;
    int v_index_ = -1;
};
```

The simulator integrates these components through the selected integrator.

## Adding A New Builtin Plugin

Builtin plugins are different from project-owned components.

High-level flow:

1. Create a plugin directory under `framework/include/gnc/plugins/<plugin_name>/`.
2. Define plugin-specific interfaces under `interfaces/`.
3. Implement builtin components under `components/`.
4. Register builtin component types in `<plugin_name>/plugin.hpp`.
5. Include the new `plugin.hpp` from `_builtin_plugins.hpp`.

Builtin component registration now happens inside `Plugin::install()`:

```cpp
class ExamplePlugin final : public gnc::core::Plugin {
public:
    const char* name() const override { return "example"; }
    gnc::core::PluginLayer layer() const override {
        return gnc::core::PluginLayer::Subsystem;
    }

    void install(gnc::core::PluginRegistry&) const override {
        auto& factory = gnc::core::ComponentFactory::instance();
        factory.registerType<ExampleComponent, IExampleInterface>(
            "example.component",
            gnc::core::ComponentCategory::Builtin,
            __FILE__);
    }
};
```

User project components should continue to use
`GNC_REGISTER_COMPONENT_TYPE`; they do not need a `Plugin` wrapper.
