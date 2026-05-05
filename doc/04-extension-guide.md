# 扩展指南

项目组件从 active project 开始：

```text
user/<active_project>/components/
```

组件头文件由 CMake 生成的显式注册链 include。新增或删除组件后需要重新运行 CMake 配置。
`components/` 只放包含 `GNC_REGISTER_COMPONENT_TYPE` 的可注册组件头。项目私有接口和共享工具头应放在 `interfaces/`、`common/` 或 `include/`，这些目录会作为 active project include path。

## 扩展前检查

先判断新能力应该放在哪里：

| 新能力 | 推荐位置 |
| --- | --- |
| 新的连续状态和导数方程 | `vehicles[].form.components` |
| 传感器、导航、制导、控制、状态机 | `vehicles[].input` 或 `vehicles[].process` |
| 气动、质量、推进、力模型、构型切换 | `vehicles[].output` |
| 批量 case 的数值扰动输入和解析状态 | `vehicles[].perturbation` |
| 把 truth、环境、命令和 output 能力转成 form input | `vehicles[].interaction.components` |
| 可复用的只读世界查询 | `environment.components` |
| 稳定的基础设施服务 | framework service package |

扩展组件进入 framework 前，应至少满足：语义稳定、接口清楚、role/stage/form family 明确、配置校验严格、测试覆盖装配失败和成功路径。否则优先留在 project component。

## 我想修改 X，该改哪里

| 目标 | 主要改动点 | 必须同步更新 |
| --- | --- | --- |
| 新增 project component | `user/<active_project>/components/` 新增可注册头；私有接口放 `interfaces/` | 重新运行 CMake；mission placement；`--list-components-verbose` 检查 role/stage/form family |
| 新增 builtin component | framework 下实现组件和接口；在 `register_builtin_packages.hpp` 注册 type id | 参考手册 type id；配置字段文档；成功和失败测试；示例 mission 片段 |
| 新增 form family | 定义 `types.hpp`、truth view、input provider、state owner 和 interaction | form-family id；`IContinuousSystem` state layout；publish 语义；observable 字段；form/interaction family mismatch 测试 |
| 新增 interaction | 读取 form truth、环境、process/output provider，生成 form input | 非空 form family；所需接口；缺依赖测试；mission placement 示例 |
| 新增 output capability | 定义或复用 provider interface，例如 aero/mass/force | `vehicle.output` placement；资产格式；配置校验；interaction 使用示例 |
| 新增 service package | 定义 service interface、package id、支持 scope、finalization task | service bootstrap；scope 错误测试；服务配置文档；说明它不是任务流程容器 |
| 修改 mission schema | 更新 `ConfigManager` / `MissionAssembler` 结构读取和诊断 | `03-mission-configuration.md`；`06-reference.md`；legacy/invalid mission 测试 |
| 修改调度语义 | 更新 `Simulator` stage/publish/integration/record 顺序 | `00-current-architecture.md`；`05-architecture.md`；ADR；publish/record/termination 语义测试 |

如果一个改动同时触碰 form、interaction 和 output，先写接口契约，再写 runtime component。不要先把所有逻辑塞进一个 interaction；这会让配置、记录和测试边界都变模糊。

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
| `vehicles[].perturbation` | `ComponentPackageRole::Perturbation` | `ExecutionStage::Perturbation` |
| `vehicles[].common` | `ComponentPackageRole::VehicleCommon` | `ExecutionStage::None` |
| `vehicles[].input` | `ComponentPackageRole::VehicleInput` | `ExecutionStage::VehicleInput` |
| `vehicles[].process` | `ComponentPackageRole::VehicleProcess` | `ExecutionStage::VehicleProcess` |
| `vehicles[].output` | `ComponentPackageRole::VehicleOutput` | `ExecutionStage::VehicleOutput` |
| `vehicles[].interaction.components` | `ComponentPackageRole::Interaction` | `ExecutionStage::Interaction` |
| `termination` | `ComponentPackageRole::Termination` | `ExecutionStage::Termination` |
| `summary` | `ComponentPackageRole::Summary` | `ExecutionStage::Summary` |

`interaction` 必须声明非空 form family，因为它是 form-specific closure。
`termination` 必须实现 `ITerminationEvaluator`；`summary` 必须实现 `ISummaryObserver`。

注册信息和 mission placement 必须一致。比如一个注册为 `VehicleOutput` / `VehicleOutput` stage 的组件放进 `vehicles[].process` 会 build fail。`vehicle.common` 是非调度层，注册 stage 必须是 `ExecutionStage::None`。

## New Form Family Contract

A project may define a new form family, including an experimental 6DoF form, but the family must be explicit. A complete form family contract contains:

- A stable form-family id, for example `contract_6dof` or `my_project_6dof`.
- Public `types.hpp`-style state, truth, and input data structures.
- A form-specific `ITruthView` interface exposed by the form component.
- A form-specific `IInputProvider` interface implemented by interaction components.
- A state owner implementing `IContinuousSystem`, with a clear `StateLayout`.
- `publish(double time)` behavior that refreshes truth/view data without integrating state.
- `IObservable` fields for the state and derived quantities the mission may record.
- A documented interaction closure that reads project process/output providers and produces the form input.

Rules enforced by the framework:

- Components placed under `vehicles[].form.components` must declare a non-empty form family.
- All form components in one vehicle must use the same form family.
- Interaction components must declare a non-empty form family and must match the selected vehicle form.
- Guidance/control components must not bypass interaction to drive a form directly.

For mixed continuous/discrete behavior, split the model: continuous state belongs in an `IContinuousSystem`; discrete sampling, commands, and mode logic belong in `vehicle.process` or `vehicle.output`; connect them with project interfaces.

Controlflow is a process-level concern. Use `gnc::libraries::StateMachine` inside a project process component when a mission needs mode or phase logic; do not model mission controlflow as a framework service.

## Perturbation Extension Contract

Perturbation is a vehicle-level source of resolved case state. A perturbation
component must implement `IPerturbationProvider`; it may also implement
`IPerturbationSnapshot` for local inspection or diagnostics. RunSet replay is
based on the generated `effective_mission.json`, so ordinary single-case replay
does not depend on any RunSet-specific snapshot file.

The intended dependency direction is one-way:

```text
RunSet numeric inputs or mission static inputs
-> vehicles[].perturbation
-> IPerturbationProvider
-> input/process/output/interaction components
```

Do not let every aero, engine, or mass component parse the RunSet file. The
RunSet format is a batch input format; the perturbation provider is the runtime
contract. This keeps single-case replay simple and lets project-specific
perturbation components map numeric inputs to strings, vectors, assets, or
flight-state-dependent values without changing the batch runner.

Components that load assets or choose behavior from perturbation state should
bind the provider in `injectDependencies()`, then read the resolved state in
`initialize()`. Do not load perturbation-dependent assets in `configure()`,
because dependency injection happens after configuration. Dynamic perturbation
values can be refreshed by the perturbation component in `update()` and read by
later stages in the same cycle.

Consumer pattern:

```cpp
#include "gnc/perturbation/interfaces/i_perturbation_provider.hpp"

class EngineModel final : public gnc::core::ComponentBase {
public:
    void injectDependencies(gnc::core::ScopedRegistry& registry) override {
        perturbation_ =
            registry.get<gnc::perturbation::IPerturbationProvider>("perturbation");
    }

    void initialize() override {
        const std::string file = perturbation_
            ? perturbation_->getString("engine.temp_level.resolved",
                                       "nominal_engine.txt")
            : "nominal_engine.txt";
        loadEngineTable(file);
    }

    void update(double) override {
        thrust_scale_ = perturbation_
            ? perturbation_->getNumber("engine.thrust_scale", 1.0)
            : 1.0;
    }

private:
    gnc::perturbation::IPerturbationProvider* perturbation_ = nullptr;
    double thrust_scale_ = 1.0;
};
```

If the component cannot run without a required perturbation item, throw a clear
error in `initialize()` after dependency injection. If the perturbation is
optional, use a physical nominal fallback as shown above. Either way, the
component should depend on the provider interface, not on a RunSet file path or
case index.

Custom perturbation components should keep their inputs numeric at the config
boundary. For string or asset choices, map integer codes to strings inside the
perturbation component, either through static config such as `enum_maps` or
through project code. For dynamic perturbation, keep the config as initial/static
case inputs and implement the flight-state-dependent logic in `update()`.

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

推荐的组件实现顺序：

1. 先定义项目私有接口，放在 `interfaces/`。
2. 实现 producer 组件，例如 process/output provider。
3. 实现 interaction，把 provider 输出转成 form input。
4. 在 mission 中装配，并用 `--list-components-verbose` 检查注册元数据。
5. 添加至少一个装配测试或示例 mission，覆盖缺依赖和正常运行。

交付前检查：

- type id 是否稳定、可读、不会和 builtin/project 其它类型冲突。
- mission placement 是否和注册 role/stage 一致。
- form-specific 组件是否声明正确 form family。
- 必填物理参数是否 build fail，而不是使用静默默认值。
- `IObservable` 字段是否是用户可以长期依赖的稳定字段。
- 组件是否只在自己的职责层做事，没有绕过 interaction 直接驱动 form。
- 文档是否说明默认 lookup name、配置字段、输出字段和失败模式。

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

## 项目自定义 Interaction 示例

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

项目组件配置建议遵循同样规则：

- 物理参数 required，不给静默默认值。
- 安全选项可以有默认值，例如 lookup name、开关、输出精度。
- 错误消息包含配置路径或字段名。
- 资产文件和 runtime component 分开：组件读取 `asset_file`，资产本身只保存数据。

## 放在 Project 还是 Framework

优先放在 project component 中，除非它语义稳定、可复用、有明确 role/stage/form family、接口契约和测试覆盖。资产、loader 和 runtime component 应保持分离。
