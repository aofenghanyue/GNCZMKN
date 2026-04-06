# Volume 3: 仿真配置与调度系统 (Simulation Configuration)

## 1. 核心调度：SimulationBuilder
`SimulationBuilder` 是连接 JSON 配置与 C++ 运行时对象的桥梁。它读取 JSON，利用 `ComponentFactory` 反射实例化类，并调用 `configure()` 注入参数。

### JSON 配置的骨架
```json
{
    "simulation": { ... },
    "outputs": { ... },
    "environment": { "components": [ ... ] },
    "vehicles": [
        {
            "id": "vehicle1",
            "components": [ ... ]
        }
    ],
    "components": [ ... ] 
}
```
*注：`components`（单飞行器模式）与 `vehicles`（多飞行器模式）可二选一。*

## 2. Simulation 节点：时间与积分器
```json
"simulation": {
    "dt": 0.01,
    "duration": 600.0,
    "integrator": "rk4",
    "stop_conditions": [ ... ]
}
```
- `dt`: 仿真全局步长（秒）。
- `duration`: 最大仿真时长（秒）。
- `integrator`: 核心推演算法，可选 `"rk4"`（四阶龙格库塔）或 `"euler"`（欧拉法）。

### 提前终止条件 (Stop Conditions)
这是 v2.0 的高级特性。您可以设置基于组件状态的安全网，避免无效计算（如坠地后继续仿真）。
```json
{
    "type": "component_field_below",
    "component": "dynamics",
    "field": "altitude",
    "value": 0.0,
    "description": "Vehicle hit the ground"
}
```
要求目标 `component` 必须实现了 `IObservable` 接口，且暴露了名为 `altitude` 的变量。支持 `component_field_below` 和 `component_field_above`。

## 3. Components 节点：工厂与配置
```json
{
    "type": "Dynamics3DOF_SphericalEarth",
    "name": "dynamics",
    "config": {
        "initial_state": {
            "altitude": 30000.0,
            "velocity": 3500.0
        }
    }
}
```
- `type`: 对应 C++ 中 `GNC_REGISTER_COMPONENT` 宏注册的类名。如果拼写错误，`SimulationBuilder` 会智能提示（Did you mean '...'?）。
- `name`: 在注册表中的实例名。其他组件通过这个名字查找它。
- `config`: 这个 JSON 对象会被封装为 `ConfigNode`，传递给组件的 `configure` 方法。
  - *安全检查*：如果组件未在代码中读取某个配置项，框架会在初始化时警告 `unrecognized config key(s)`，帮您捕捉拼写错误。

## 4. ConfigNode API 快速参考
在 C++ 组件中读取配置：
```cpp
void configure(const gnc::core::ConfigNode& config) override {
    // asDouble(默认值)
    mass_ = config["mass_kg"].asDouble(100.0);
    
    // asBool, asInt, asString
    enabled_ = config["enabled"].asBool(true);
    
    // 遍历数组
    const auto& arr = config["table"];
    for(size_t i=0; i<arr.size(); ++i) {
        val_[i] = arr[i].asDouble();
    }
}
```