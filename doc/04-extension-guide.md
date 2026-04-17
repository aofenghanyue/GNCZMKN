# 扩展指南

扩展框架时先判断你要加的是项目组件、内置组件，还是服务。三者的注册方式不同。

## 新增项目组件

项目组件放在：

```text
user/<project>/components/
```

CMake 会把活动项目的组件头文件包含进 `auto_registered_components.hpp`。组件用 `GNC_REGISTER_COMPONENT_TYPE` 注册，不需要写插件。

一个最小组件如下：

```cpp
#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"

class MyComponent final : public gnc::core::ComponentBase {
public:
    MyComponent() : ComponentBase("MyComponent") {}

    void configure(const gnc::core::ConfigNode& config) override {
        value_ = config["value"].asDouble(value_);
    }

    void update(double dt) override {
        value_ += dt;
    }

private:
    double value_ = 0.0;
};

GNC_REGISTER_COMPONENT_TYPE("example.my_component", MyComponent)
```

任务里这样引用：

```json
{
  "type": "example.my_component",
  "name": "my_component",
  "config": {
    "value": 1.0
  }
}
```

`type` 是注册给工厂的 type id。`name` 是实体内局部名。装配后完整名由实体前缀生成。

## 绑定组件依赖

组件依赖其他组件时，优先依赖接口。

```cpp
void injectDependencies(gnc::core::ScopedRegistry& registry) override {
    registry.bindAll(gnc::core::bind(state_solver_, "dynamics"));
}
```

`"dynamics"` 没有点号，`ScopedRegistry` 会把它解析为当前实体内的 `<entity_id>.dynamics`。跨实体依赖要写完整名：

```cpp
registry.bindAll(gnc::core::bind(atmosphere_, "env.atmosphere"));
```

可选依赖使用 `bindIfPresent`。必需依赖使用 `bind`，构建阶段会做预检，失败时给出同作用域组件和候选提供者。

## 暴露可观测字段

实现 `IObservable` 后，`AutoDataLogger` 才能记录组件字段。

```cpp
#include "gnc/infrastructure/observable_helpers.hpp"
#include "gnc/interfaces/i_observable.hpp"

std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
    gnc::core::ObservableFieldBuilder builder;
    builder.addScalar("value", [this]() { return value_; });
    return builder.build();
}
```

字段名应该稳定。临时排查数据可以用 `snapDebug()` 写调试快照，不要把临时字段混进稳定输出。

## 实现连续系统

如果组件状态需要积分器推进，实现 `IContinuousSystem`：

```cpp
class Dynamics final : public gnc::core::ComponentBase,
                       public gnc::interfaces::IContinuousSystem {
public:
    const gnc::core::StateLayout& getStateLayout() const override;
    void computeDerivatives(double time,
                            const Eigen::VectorXd& state,
                            Eigen::VectorXd& derivative) const override;
    const Eigen::VectorXd& getState() const override;
    void setState(const Eigen::VectorXd& state) override;
    Eigen::VectorXd getInitialState() const override;
};
```

仿真器每步会读取状态、调用积分器、写回状态，再调用组件的 `update(dt)`。连续系统的 `update()` 适合做积分后的缓存或输出更新，不适合再次推进主状态。

## 使用服务

服务通过 `injectServices()` 获取：

```cpp
void injectServices(gnc::core::ServiceContext& services) override {
    coord_service_ = services.get<gnc::plugins::soviet_coord::ISovietCoordService>();
}
```

如果服务是必需能力，组件应在 `update()` 或 `initialize()` 前检查指针，或者使用 `require` 风格的接口封装。服务适合表达基础设施能力，比如坐标变换；会被仿真器调度的模型仍应写成组件。

## 新增内置插件组件

内置组件放在 `framework/include/gnc/plugins/<plugin_name>/`。典型步骤：

1. 定义接口，放在 `interfaces/`。
2. 定义组件实现，放在 `components/`。
3. 在 `plugin.hpp` 的 `Plugin::install()` 中调用 `ComponentFactory::registerType`。
4. 如果是新插件，把 `plugin.hpp` 加到 `framework/include/gnc/plugins/_builtin_plugins.hpp`。
5. 确认插件依赖顺序和 `dependencies()` 一致。

内置组件注册示例：

```cpp
void install(gnc::core::PluginRegistry&) const override {
    auto& factory = gnc::core::ComponentFactory::instance();
    factory.registerType<MyBuiltinComponent, IMyInterface>(
        "plugin_name.my_component",
        gnc::core::ComponentCategory::Builtin,
        __FILE__);
}
```

项目组件继续使用 `GNC_REGISTER_COMPONENT_TYPE`。不要为了项目组件新增插件。

## 新增服务安装器

服务安装器注册在插件里：

```cpp
registry.registerServiceInstaller(
    "my_service",
    [](const gnc::core::PluginRegistry::ServiceInstallRequest& request) {
        auto service = std::make_shared<MyService>();
        request.services.registerService<IMyService>(service);
    });
```

如果服务需要绑定组件，使用 `request.deferred_actions`。这样可以等所有实体组件注册完成后再查找依赖。

## 扩展边界

- 项目算法优先写在 `user/<project>/components/`。
- 可复用的物理模型或公共基础设施再提升为内置插件。
- 组件之间通过接口通信，不要直接包含对方具体类。
- 任务配置里不要依赖声明顺序以外的隐式全局状态。
- 组件名、接口名和配置键要稳定，日志和停止条件会直接引用它们。

