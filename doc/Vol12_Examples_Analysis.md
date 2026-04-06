# Volume 12: 复杂案例分析 (Examples Analysis)

我们通过官方的 03 号案例 `examples/03_cavh_3dof` 来展示框架的强大之处。这是一个**高超声速飞行器（CAVH）的 3DOF 弹道仿真**。

## 1. 业务逻辑分离 (Components)
在这个案例中，作者没有把气动、质量、制导和动力学揉在一个类里，而是充分利用了框架的接口。

- **`CavhMass`**: 实现了 `IMassProperty`。虽然当前返回固定质量 900kg，但未来加入耗油逻辑时，只需修改此类，完全不影响动力学积分。
- **`CavhProgrammedAoA`**: 实现了 `IGuidance3DOF`。
  - **核心逻辑**：根据高度查表输出攻角（AoA）。
  - **巧妙之处**：它通过依赖注入获取了 `IDynamicsModel`，在 `update` 中使用 `dynamics_->getStateValue("altitude")` 获取高度。
- **`CavhAerodynamics`**: 实现了 `IAeroCoefficients`。
  - **核心逻辑**：气动系数计算。$C_L = C_{L0} + C_{L\alpha} \cdot \alpha$。
  - **巧妙之处**：它同时依赖了 `IGuidance3DOF`（获取指令攻角）和 `IDynamicsModel`（获取速度计算马赫数）。这种网状的“主动拉取”数据流极其清晰。

## 2. 无缝接入动力学
```json
{
    "type": "Dynamics3DOF_SphericalEarth",
    "name": "dynamics",
    "config": {
        "initial_state": { "altitude": 60000.0, "velocity": 3200.0 }
    }
}
```
`Dynamics3DOF_SphericalEarth` 作为核心的动力学积分器，会自动查找上述所有组件。如果没有注册 `aero`，它假设气动力为 0；如果找到了，它会在积分导数时调用 `aero_->computeCoefficients()`。

## 3. 安全的运行边界
在 `cavh_mission.json` 中配置了提前终止：
```json
"stop_conditions": [
    {
        "type": "component_field_below",
        "component": "dynamics",
        "field": "altitude",
        "value": 10000.0
    }
]
```
因为 3DOF 的球地模型在高度 $<0$ 时会报错（除零异常或气动模型超限），框架会在飞行器降落到 10km 时自动且安全地停止仿真。

## 4. 可观测性展现
配置文件中启用了极细粒度的数据收集：
```json
"record": {
    "dynamics": ["altitude", "velocity", "mach", "lift_to_drag", "alpha_deg"],
    "guidance": "all"
}
```
`AutoDataLogger` 在底层工作，生成了高精度的 CSV 文件。运行结束后，您可以直接使用 `tools/plot_results.py user/outputs/.../cavh_3dof.csv` 绘制出高度与速度随时间变化的平滑曲线，分析气动模型的表现。

这就是 GNC 仿真框架 v2.0 所倡导的：**高内聚、低耦合、配置驱动、自动遥测**。