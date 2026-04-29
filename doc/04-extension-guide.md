# 扩展指南

项目组件从 active project 开始：

```text
user/<active_project>/components/
```

组件头文件由 CMake 生成的显式注册链 include。新增或删除组件后需要重新运行 CMake 配置。
`components/` 只放包含 `GNC_REGISTER_COMPONENT_TYPE` 的可注册组件头。项目私有接口和共享工具头应放在 `interfaces/`、`common/` 或 `include/`，这些目录会作为 active project include path。

## 注册

项目组件使用：

```cpp
GNC_REGISTER_COMPONENT_TYPE(TypeId,
                            ComponentType,
                            PackageRole,
                            ExecutionStage,
                            FormFamily,
                            Interfaces...)
```

常用 placement：

| Mission 位置 | Role | Stage |
| --- | --- | --- |
| `environment.components` | `ComponentPackageRole::Environment` | `ExecutionStage::Environment` |
| `vehicles[].form.components` | `ComponentPackageRole::Form` | `ExecutionStage::Form` |
| `vehicles[].common` | `ComponentPackageRole::VehicleCommon` | `ExecutionStage::None` |
| `vehicles[].input` | `ComponentPackageRole::VehicleInput` | `ExecutionStage::VehicleInput` |
| `vehicles[].process` | `ComponentPackageRole::VehicleProcess` | `ExecutionStage::VehicleProcess` |
| `vehicles[].output` | `ComponentPackageRole::VehicleOutput` | `ExecutionStage::VehicleOutput` |
| `vehicles[].interaction.components` | `ComponentPackageRole::Interaction` | `ExecutionStage::Interaction` |
| `termination` | `ComponentPackageRole::Termination` | `ExecutionStage::Termination` |
| `summary` | `ComponentPackageRole::Summary` | `ExecutionStage::Summary` |

`interaction` 必须声明非空 form family，因为它是 form-specific closure。
`termination` 必须实现 `ITerminationEvaluator`；`summary` 必须实现 `ISummaryObserver`。

## 生命周期

普通离散组件主要实现 `update(double dt)`：

```text
update 在每个周期最多调用一次。
update 读取 t_k 发布态。
update 产生用于 [t_k, t_{k+1}] 的本周期离散输出。
update 不负责连续状态积分。
```

连续状态组件实现 `IContinuousSystem::computeDerivatives()`。如果组件还需要刷新 truth/view/observer 派生量，应实现 `publish(double time)`，不要在 `update()` 中再次积分。

`before_step(step, t_k, dt)` 在离散 update 前调用。`after_step(step, t_{k+1}, dt)` 是低层钩子，发生在积分提交之后、下一周期 publish 之前；普通用户不应依赖它修改记录/停止语义。

publish 排序先看 phase，再看 priority。`IContinuousSystem` 组件按 state owner 发布；view / observer 组件应覆盖 `getPublishPhase()` 并返回 `PublishPhase::View`；普通组件保持默认。mission 中的可选 `priority` 必须是整数式 number，并且只在同一 phase 或 execution stage 内排序。`rate_hz` 必须严格整除仿真频率，不会隐式 round。record 记录的是各组件在该时刻暴露的当前 observable；物理延迟语义由组件自身定义。

## 最小组件示例

```cpp
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"

#include <vector>

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

mission 中放入 `vehicles[].process`：

```json
{
  "vehicles": [
    {
      "id": "vehicle",
      "form": { "components": [] },
      "common": [],
      "input": [],
      "process": [
        {
          "type": "example.step_counter",
          "name": "counter",
          "config": { "increment": 2.0 }
        }
      ],
      "output": [],
      "interaction": { "components": [] }
    }
  ]
}
```

## Project Custom Interaction Example

Project-specific interactions should keep contracts in `interfaces/`, command
production in `process`, and form closure in `interaction.components`.

Project interface:

```cpp
// user/my_project/interfaces/i_lateral_command.hpp
#pragma once

struct LateralCommand {
    double acceleration_y_mps2 = 0.0;
};

class ILateralCommandProvider {
public:
    virtual ~ILateralCommandProvider() = default;
    virtual LateralCommand lateralCommand() const = 0;
};
```

Process command provider:

```cpp
// user/my_project/components/lateral_command.hpp
#pragma once

#include "interfaces/i_lateral_command.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"

class LateralCommandComponent final : public gnc::core::ComponentBase,
                                      public ILateralCommandProvider {
public:
    LateralCommandComponent() : ComponentBase("LateralCommandComponent") {}

    void update(double) override {
        command_.acceleration_y_mps2 = acceleration_y_mps2_;
    }

    LateralCommand lateralCommand() const override { return command_; }

private:
    double acceleration_y_mps2_ = 2.0;
    LateralCommand command_{};
};

GNC_REGISTER_COMPONENT_TYPE("project.process.lateral_command",
                            LateralCommandComponent,
                            ::gnc::core::ComponentPackageRole::VehicleProcess,
                            ::gnc::core::ExecutionStage::VehicleProcess,
                            "",
                            ILateralCommandProvider)
```

Interaction closure:

```cpp
// user/my_project/components/cartesian_lateral_interaction.hpp
#pragma once

#include "interfaces/i_lateral_command.hpp"
#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/core/scoped_registry.hpp"
#include "gnc/forms/cartesian_3dof/interfaces/i_input_provider.hpp"

#include <stdexcept>

class CartesianLateralInteraction final
    : public gnc::core::ComponentBase,
      public gnc::forms::cartesian_3dof::IInputProvider {
public:
    CartesianLateralInteraction() : ComponentBase("CartesianLateralInteraction") {}

    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        command_ = registry.get<ILateralCommandProvider>("lateral_command");
        if (!command_) {
            throw std::runtime_error("Missing lateral_command provider.");
        }
    }

    void update(double) override {}

    gnc::forms::cartesian_3dof::Input computeCartesian3DoFInput(
        const gnc::forms::cartesian_3dof::Truth&,
        double) const override {
        const auto command = command_->lateralCommand();
        gnc::forms::cartesian_3dof::Input input;
        input.acceleration_mps2.y() = command.acceleration_y_mps2;
        return input;
    }

private:
    ILateralCommandProvider* command_ = nullptr;
};

GNC_REGISTER_COMPONENT_TYPE("project.interaction.cartesian_lateral",
                            CartesianLateralInteraction,
                            ::gnc::core::ComponentPackageRole::Interaction,
                            ::gnc::core::ExecutionStage::Interaction,
                            "cartesian_3dof",
                            gnc::forms::cartesian_3dof::IInputProvider)
```

Mission placement:

```json
{
  "vehicles": [
    {
      "id": "vehicle",
      "process": [
        {
          "type": "project.process.lateral_command",
          "name": "lateral_command",
          "rate_hz": 10.0,
          "config": {}
        }
      ],
      "interaction": {
        "components": [
          {
            "type": "project.interaction.cartesian_lateral",
            "name": "interaction",
            "config": {}
          }
        ]
      }
    }
  ]
}
```

## 配置读取

Framework builtin 应使用 `ConfigReader` 做 required 字段、类型和 unknown-key 校验。项目组件可以继续直接读取 `ConfigNode`，但建议对物理参数也显式校验，避免拼错字段后静默运行。

## 放在 Project 还是 Framework

优先放在 project component 中，除非它语义稳定、可复用、有明确 role/stage/form family、接口契约和测试覆盖。资产、loader 和 runtime component 应保持分离。
