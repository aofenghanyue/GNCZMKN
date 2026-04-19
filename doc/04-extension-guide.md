# 扩展指南

这篇文档先解决“5 分钟内如何跑通第一个组件”，再补连续系统、服务安装器和内置插件扩展。

## 5 分钟跑通第一个项目组件

下面用一个最小离散组件贯穿完整流程：

1. 新建组件头文件
2. 注册 type id
3. 在 mission 里引用它
4. 编译并运行
5. 看输出和诊断

### 第 1 步：新建文件

把文件放在：

```text
user/<project>/components/
```

例如：

```text
user/example_02_atmospheric_3dof/components/step_counter.hpp
```

写入下面这个最小组件：

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

GNC_REGISTER_COMPONENT_TYPE("example.step_counter", StepCounter, gnc::interfaces::IObservable)
```

这段代码已经包含了一个项目组件最常见的三件事：

- 读 `config`
- 在 `update()` 里推进内部状态
- 通过 `IObservable` 暴露稳定输出

### 第 2 步：在 mission 里引用它

项目组件不需要额外写插件。CMake 会把活动项目下的组件头文件自动收进 `auto_registered_components.hpp`。

在 `entities[].components` 里加入：

```json
{
  "type": "example.step_counter",
  "name": "counter",
  "config": {
    "increment": 2.0
  }
}
```

一个最小可运行 mission 片段如下：

```json
{
  "simulation": {
    "dt": 0.1,
    "duration": 1.0
  },
  "outputs": {
    "session_name": "counter_demo",
    "record": {
      "demo.counter": "all"
    }
  },
  "entities": [
    {
      "id": "demo",
      "role": "vehicle",
      "components": [
        {
          "type": "example.step_counter",
          "name": "counter",
          "config": {
            "increment": 2.0
          }
        }
      ]
    }
  ]
}
```

注意：

- `type` 是注册给工厂的 type id。
- `name` 是实体内局部名。
- 装配后的完整名是 `demo.counter`，不是 `counter`。

### 第 3 步：编译并运行

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles"
cmake --build build-mingw -j 4
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

如果想先确认组件是否成功注册，可以运行：

```powershell
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

你应能看到：

- `example.step_counter`
- 它暴露的接口，例如 `IObservable`
- 注册来源文件路径

### 第 4 步：看输出和诊断

如果 `outputs.record` 配好了，CSV 里会出现 `demo.counter.count`。

如果构建失败，优先看构建期诊断。当前常见失败原因有：

- `type` 拼错，找不到已注册组件
- `entities[]` 缺失或写错类型
- 组件 `config` 里出现未识别字段
- 依赖绑定失败

### 第 5 步：理解真实生命周期

当前真实顺序是：

1. `configure`
2. `injectServices`
3. 注册到 `ComponentRegistry`
4. build 期依赖预检 `injectDependencies`
5. `initialize`
6. `update`
7. `finalize`

这里有两个容易搞混的点：

- `injectServices()` 发生在构建期，不是在 `Simulator::initialize()` 之后。
- `injectDependencies()` 会在构建期先做一次预检；如果预检已成功，运行时不会重复注入。

## 组件顺序与依赖顺序

组件依赖解析不靠 `components[]` 声明顺序，因为构建结束后会统一做依赖预检。

但运行时 `update()` 顺序就是 `components[]` 的声明顺序。因此：

- 如果组件 A 在同一步读取组件 B 刚刚更新出的量，B 应排在 A 前面。
- 如果只是静态绑定，不依赖“同一步谁先更新”，顺序通常不影响能否绑定成功。

## 组件依赖绑定

组件优先依赖接口，不直接依赖具体实现类。

```cpp
void injectDependencies(gnc::core::ScopedRegistry& registry) override {
    registry.bindAll(gnc::core::bind(state_solver_, "dynamics"));
    registry.bindAll(gnc::core::bind(atmosphere_, "env.atmosphere"));
}
```

规则如下：

- 没有点号的 `"dynamics"` 会被解释为当前实体里的 `<entity_id>.dynamics`
- 跨实体依赖要写完整名，例如 `env.atmosphere`
- 必需依赖用 `bind`
- 可选依赖用 `bindIfPresent`

构建期会做依赖预检。若失败，诊断里会给出：

- 同作用域组件列表
- 目标接口的候选提供者
- 可能的拼写建议

## 服务注入

服务通过 `injectServices()` 获取：

```cpp
void injectServices(gnc::core::ServiceContext& services) override {
    coord_service_ = services.get<gnc::plugins::soviet_coord::ISovietCoordService>();
}
```

服务注入顺序固定为：

```text
global -> environment -> vehicle
```

因此，飞行器组件既能拿到全局服务，也能拿到环境和本实体局部服务。

## 暴露稳定输出与临时调试

暴露稳定输出时，实现 `IObservable`：

```cpp
std::vector<gnc::interfaces::ObservableField> getObservableFields() const override {
    gnc::core::ObservableFieldBuilder builder;
    builder.addScalar("value", [this]() { return value_; });
    return builder.build();
}
```

临时排查时，用 `snapDebug()`：

```cpp
void update(double dt) override {
    residual_ *= 0.5;
    snapDebug("solver.residual", residual_);
}
```

边界建议：

- 会长期保留并进入分析链路的量，放 `IObservable`
- 只为排障的量，放 `snapDebug`

## 进阶：连续系统

如果组件状态要由积分器推进，实现 `IContinuousSystem`：

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

运行时每一步会：

1. 读取状态
2. 调积分器
3. 写回状态
4. 再调组件的 `update(dt)`

因此，连续系统的 `update()` 适合做积分后的缓存刷新、输出同步和调试快照，不适合再次推进主状态。

## 进阶：新增服务安装器

服务安装器在插件里注册：

```cpp
registry.registerServiceInstaller(
    "my_service",
    [](const gnc::core::PluginRegistry::ServiceInstallRequest& request) {
        auto service = std::make_shared<MyService>();
        request.services.registerService<IMyService>(service);
    });
```

如果服务需要在所有组件注册完成后再绑定组件，使用 `request.deferred_actions`。

## 进阶：新增内置插件组件

内置组件放在：

```text
framework/include/gnc/plugins/<plugin_name>/
```

典型步骤：

1. 在 `interfaces/` 下定义接口
2. 在 `components/` 下写实现
3. 在 `plugin.hpp` 的 `install()` 里调用 `ComponentFactory::registerType`
4. 如果是新插件，把 `plugin.hpp` 加到 `_builtin_plugins.hpp`

注册示例：

```cpp
void install(gnc::core::PluginRegistry&) const override {
    auto& factory = gnc::core::ComponentFactory::instance();
    factory.registerType<MyBuiltinComponent, IMyInterface>(
        "plugin_name.my_component",
        gnc::core::ComponentCategory::Builtin,
        __FILE__);
}
```

## 扩展边界

- 项目算法优先写在 `user/<project>/components/`
- 可复用的物理模型或基础能力再提升为内置插件
- 组件之间通过接口通信，不要直接包含对方具体类
- 配置、日志和诊断里的名字要保持稳定，因为这些名字会被停机条件、脚本和用户操作直接引用
