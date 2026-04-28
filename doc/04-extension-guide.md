# 扩展指南

项目组件从 active project 开始：

```text
user/<active_project>/components/
```

组件头文件由 CMake 生成的显式注册链 include。新增或删除组件后需要重新运行 CMake 配置。

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

`interaction` 必须声明非空 form family，因为它是 form-specific closure。

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

publish 排序先看 phase，再看 priority。`IContinuousSystem` 组件按 state owner 发布；view / observer 组件应覆盖 `getPublishPhase()` 并返回 `PublishPhase::View`；普通组件保持默认。mission 中的可选 `priority` 必须是整数式 number，并且只在同一 phase 或 execution stage 内排序。record 记录的是各组件在该时刻暴露的当前 observable；物理延迟语义由组件自身定义。

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

## 配置读取

Framework builtin 应使用 `ConfigReader` 做 required 字段、类型和 unknown-key 校验。项目组件可以继续直接读取 `ConfigNode`，但建议对物理参数也显式校验，避免拼错字段后静默运行。

## 放在 Project 还是 Framework

优先放在 project component 中，除非它语义稳定、可复用、有明确 role/stage/form family、接口契约和测试覆盖。资产、loader 和 runtime component 应保持分离。
