# GNCZMKN

GNCZMKN 是一个面向制导、导航与控制研究的轻量 C++ 仿真框架。它的目标不是通用商用仿真平台，而是让研究人员用清晰的组件边界快速建立可信模型、复现实验、扩展项目私有模型。

适合使用它的场景：

- 固定步长 GNC 算法、飞行动力学、传感器/制导/控制链路原型验证。
- 需要把项目模型作为组件接入，并保留可测试的接口边界。
- 需要通过 mission JSON 显式装配仿真，而不是把场景逻辑硬编码进 runner。

当前 mission 只推荐一种顶层写法：`vehicles[]`。旧式 `entities[]`、根级 `components/services`、根级 `form/vehicle/interaction` 都不是当前运行时契约。

## 核心心智模型

| 概念 | 放什么 | 不放什么 |
| --- | --- | --- |
| `Form` | 连续状态、导数方程、truth view、form input | 导航估计、制导控制策略 |
| `Environment` | 地球、大气、重力等只读世界查询 | 飞行器状态、任务模式 |
| `Vehicle` | `common/input/process/output` 飞行器侧能力 | form-specific 闭合方程 |
| `Interaction` | 把 truth、环境、process 命令和 output 能力闭合成 form input | 气动表、质量定义、推进资产 |

固定步长循环采用周期开始发布态：

```text
publish/refresh t_k -> before_step(t_k) -> discrete update(t_k)
-> record t_k -> stop check t_k
-> synchronized independent integration to t_{k+1}
-> next cycle
```

CSV 中的 `time` 列表示该行发布态的物理时间。form/dynamics/truth view 字段是周期开始发布态 `x_k` 及其真实派生量；input/process/guidance/output 字段是 `update(t_k)` 后各组件暴露的本周期离散输出。默认第一行是 `x_0` 加上基于 `x_0` 计算出的第一周期离散输出；触发停止条件的发布态会先写入 CSV，再终止仿真。

## Quick Start

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DBUILD_TESTS=ON
cmake --build build-mingw -j 4
ctest --test-dir build-mingw --output-on-failure
```

运行当前 active project：

```powershell
build-mingw\bin\gnc_sim.exe
```

显式运行示例：

```powershell
build-mingw\bin\gnc_sim.exe --config user/example_02_atmospheric_3dof/config/mission.json
```

查看已注册组件：

```powershell
build-mingw\bin\gnc_sim.exe --list-components-verbose
```

## Mission Skeleton

下面是可运行的最小 Cartesian 3DoF mission 形状；完整示例见 `user/example_01_minimal_pluginized/config/mission.json`。

```json
{
  "simulation": {
    "dt": 0.1,
    "duration": 10.0,
    "integrator": "rk4"
  },
  "environment": {},
  "vehicles": [
    {
      "id": "vehicle",
      "form": {
        "components": [
          {
            "type": "form.cartesian_3dof.point_mass",
            "name": "dynamics",
            "config": {
              "initial_position": [0.0, 0.0, 1000.0],
              "initial_velocity": [250.0, 0.0, 40.0]
            }
          }
        ]
      },
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": {
        "components": [
          {
            "type": "interaction.cartesian_3dof.direct_accel",
            "name": "interaction",
            "config": {
              "acceleration_mps2": [0.0, 0.0, -9.81]
            }
          }
        ]
      }
    }
  ],
  "outputs": {
    "directory": "user/outputs/{timestamp}",
    "format": "csv",
    "session_name": "minimal_cartesian_3dof",
    "record": {
      "vehicle.dynamics": "all"
    }
  },
  "termination": {
    "type": "termination.component_field_below",
    "name": "termination",
    "config": {
      "component": "vehicle.dynamics",
      "field": "altitude",
      "value": 0.0
    }
  }
}
```

`simulation.dt` 和 `simulation.duration` 必须显式配置；`integrator` 可省略，默认 `rk4`。物理参数如初始状态、质量、气动表和控制表不会静默使用默认值。

组件条目可以增加可选 `priority` 和 `rate_hz`；框架仍先应用固定 stage / publish phase，再在同一组内按 priority 排序。`rate_hz` 必须严格整除仿真频率。

## Repository Layout

| 路径 | 用途 |
| --- | --- |
| `framework/include/gnc/core/` | 运行时外壳、装配、配置、验证、调度 |
| `framework/include/gnc/forms/` | 内置 form 和 form 专属接口 |
| `framework/include/gnc/environment/` | 环境组件和查询接口 |
| `framework/include/gnc/vehicle/` | vehicle common/input/process/output 包 |
| `framework/include/gnc/interactions/` | form-aware interaction 包 |
| `framework/include/gnc/services/` | service package |
| `framework/data/` | 仿真资产 |
| `user/` | active project、示例 mission、项目组件和输出 |
| `tests/` | 架构契约、组件、mission、日志和时间语义测试 |
| `doc/` | 用户和维护文档 |

## Documentation

- 第一次运行：读 [文档导航](doc/README.md) 和 [快速上手](doc/01-getting-started.md)。
- 理解架构全貌：按 [核心概念](doc/02-core-concepts.md)、[当前架构总览](doc/00-current-architecture.md)、[维护者架构说明](doc/05-architecture.md)、[扩展指南](doc/04-extension-guide.md) 的顺序读。
- 写 mission：读 [核心概念](doc/02-core-concepts.md)、[Mission 配置](doc/03-mission-configuration.md) 和 [参考手册](doc/06-reference.md)。
- 写项目组件：读 [扩展指南](doc/04-extension-guide.md) 和 `user/README.md`。
- 维护框架：读 [当前架构总览](doc/00-current-architecture.md)、[维护者架构说明](doc/05-architecture.md) 和 [设计决策](doc/07-decisions.md)。
