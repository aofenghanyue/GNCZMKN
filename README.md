# GNCZMKN

GNCZMKN 是一个面向制导、导航与控制研究的轻量 C++ 仿真框架。它的目标不是通用商用仿真平台，而是让研究人员用清晰的组件边界快速建立可信模型。

当前 mission 只推荐一种顶层写法：`vehicles[]`。旧式 `entities[]`、根级 `components/services`、根级 `form/vehicle/interaction` 都不是当前运行时契约。

## 核心概念

- `Form`: 拥有连续状态、导数方程、truth view 和 form input。
- `Environment`: 提供地球、大气、重力等只读环境查询能力。
- `Vehicle`: 按 `common/input/process/output` 组织飞行器侧能力。
- `Interaction`: 把 form truth、环境查询、process 命令和 output 能力闭合成 form input。

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
      "form": { "components": [] },
      "services": {},
      "common": [],
      "input": [],
      "process": [],
      "output": [],
      "interaction": { "components": [] }
    }
  ],
  "outputs": {},
  "stop_conditions": []
}
```

`simulation.dt` 和 `simulation.duration` 必须显式配置；`integrator` 可省略，默认 `rk4`。物理参数如初始状态、质量、气动表和控制表不会静默使用默认值。

组件条目可以增加可选 `priority`；框架仍先应用固定 stage / publish phase，再在同一组内按 priority 排序。
## Repository Layout

| 路径 | 用途 |
| --- | --- |
| `framework/include/gnc/core/` | 运行时外壳、装配、配置、验证、停止条件 |
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

- [文档导航](doc/README.md)
- [快速上手](doc/01-getting-started.md)
- [核心概念](doc/02-core-concepts.md)
- [Mission 配置](doc/03-mission-configuration.md)
- [扩展指南](doc/04-extension-guide.md)
- [参考手册](doc/06-reference.md)
- [当前架构总览](doc/00-current-architecture.md)
- [维护者架构说明](doc/05-architecture.md)
- [设计决策](doc/07-decisions.md)
