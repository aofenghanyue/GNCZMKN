# 扩展指南

这篇文档说明如何给当前框架新增组件，以及何时把代码放在项目侧、framework builtin 或 service package 中。

## Active Project

项目侧扩展从 active project 开始：

```text
user/active_project
```

文件内容是 `user/` 下的项目目录名，例如：

```text
example_02_atmospheric_3dof
```

当前构建只会自动扫描 active project 的组件目录：

```text
user/<active_project>/components/
```

修改 `user/active_project`、新增组件头文件或删除组件头文件后，需要重新运行 CMake 配置和构建。

## 项目组件目录

推荐项目布局：

```text
user/<project>/
  components/
    my_component.hpp
  config/
    mission.json
```

组件头文件会被 CMake 生成的显式注册链 include。不要依赖静态初始化或运行时动态发现。

旧目录 `user/components/` 只是过渡目录，不是当前推荐扩展入口。

## 注册宏

项目组件使用：

```cpp
GNC_REGISTER_COMPONENT_TYPE(TypeId,
                            ComponentType,
                            PackageRole,
                            ExecutionStage,
                            FormFamily,
                            Interfaces...)
```

必须明确声明：

| 参数 | 含义 |
| --- | --- |
| `TypeId` | mission 中 `type` 使用的字符串 |
| `ComponentType` | 组件 C++ 类型 |
| `PackageRole` | 组件应放入哪个 mission 块 |
| `ExecutionStage` | 组件每步在哪个 stage 执行，或 `None` |
| `FormFamily` | 适配的 form family，form-neutral 时用 `""` |
| `Interfaces...` | 对外暴露的接口，例如 `IObservable` |

如果注册宏不在构建生成的注册链中，编译会失败。这是有意设计，用来避免隐藏注册路径。

## Role 和 Stage

常用组合：

| 放置位置 | Role | Stage |
| --- | --- | --- |
| `environment.components` | `ComponentPackageRole::Environment` | `ExecutionStage::Environment` |
| `form.components` | `ComponentPackageRole::Form` | `ExecutionStage::Form` |
| `vehicle.common` | `ComponentPackageRole::VehicleCommon` | `ExecutionStage::None` |
| `vehicle.input` | `ComponentPackageRole::VehicleInput` | `ExecutionStage::VehicleInput` |
| `vehicle.process` | `ComponentPackageRole::VehicleProcess` | `ExecutionStage::VehicleProcess` |
| `vehicle.output` | `ComponentPackageRole::VehicleOutput` | `ExecutionStage::VehicleOutput` |
| `interaction.components` | `ComponentPackageRole::Interaction` | `ExecutionStage::Interaction` |

assembly 会校验放置位置和注册元数据。比如 `vehicle.common` 不允许注册了运行时 stage 的组件。

## Form Family

form-specific 组件需要声明 form family。例如：

- Cartesian 3DoF: `"cartesian_3dof"`
- Local-spherical 3DoF: `"local_spherical_3dof"`
- Form-neutral: `""`

mission 选择的 form family 来自 form 组件，或者来自 `form.family`。如果 interaction 或 output 组件声明了不兼容的 form family，assembly 会失败。

## 最小组件示例

下面的组件适合放在 `user/<active_project>/components/step_counter.hpp`：

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

mission 中放入：

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

构建后用：

```powershell
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

确认 `example.step_counter` 出现在 Project/example components 列表里。

## 放在 Project 还是 Framework

优先放在 project component 中，当它：

- 只服务某个任务或某个用户项目。
- 仍在实验或快速迭代。
- 不需要成为稳定 builtin type id。

考虑移动到 framework builtin，当它：

- 可复用且语义稳定。
- 有清晰的 package role、stage、form family 和接口契约。
- 有测试覆盖，并且不依赖某个示例项目的私有假设。

framework builtin 应放入对应包目录：

```text
framework/include/gnc/
  forms/
  environment/
  vehicle/common/
  vehicle/input/
  vehicle/process/
  vehicle/output/
  interactions/
```

然后通过 `gnc/bootstrap/register_builtin_packages.hpp` 的显式 bootstrap 注册。

## Service Package

服务不是普通组件。服务由 `ServicePackageRegistry` 管理，负责声明 service id、支持作用域、创建逻辑和 finalization task。

当前 `coordinate_tree` 是 vehicle-scoped v1 服务：

```text
vehicles[].services.coordinate_tree
```

不要把 coordinate-tree spec 当作项目组件自动注册。当前 specs 由 coordinate-tree 服务包拥有：

```text
framework/include/gnc/services/coordinate_tree/specs/
framework/include/gnc/services/coordinate_tree/bootstrap/register_service_package.hpp
```

如果要新增内置 spec，应在 service package 边界内注册，不要把具体 spec 清单写进 `SimulationBuilder` 或 generic assembler。

## 扩展检查清单

新增组件后检查：

- 头文件位于 `user/<active_project>/components/` 或正确的 framework 包目录。
- 注册 type id 与 mission 中的 `type` 完全一致。
- role/stage 与 mission 放置位置一致。
- form family 与 mission 选择的 form 兼容。
- 需要记录的数据通过 `IObservable` 暴露。
- 重新配置并构建后，`--list-components-verbose` 能看到新 type id。
